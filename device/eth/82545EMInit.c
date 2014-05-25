/* 82545EMInit.c - _82545EMInit */

#include <xinu.h>

local 	status 	_82545EM_init_hw(struct ether *);
local 	void 	_82545EM_reset_hw(struct ether *);
local 	void 	_82545EM_configure_rx(struct ether *);
local 	void 	_82545EM_configure_tx(struct ether *);
status 	_82545EM_read_phy_reg(struct ether *, uint32, uint16 *);
status 	_82545EM_write_phy_reg(struct ether *, uint32, uint16);


/*------------------------------------------------------------------------
 * _82545EMInit - initialize Intel 82545EM Ethernet NIC
 *------------------------------------------------------------------------
 */
status 	_82545EMInit(
	struct 	ether *ethptr
	)
{
	uint16	command;
	int32	i;
	uint32	rar_low, rar_high;
	
	struct e1000_rx_desc *rxRingPtr;	/* receive descriptor */
	struct e1000_tx_desc *txRingPtr;	/* transmit descriptor */
	uint32 buffer_ptr;
	/* Read PCI configuration information */
	/* Read I/O base address */
	pci_bios_read_config_dword(ethptr->pcidev, E1000_PCI_IOBASE, (uint32 *)&ethptr->iobase);
	ethptr->iobase &= ~1;
	ethptr->iobase &= 0xffff;
	
	pci_bios_read_config_dword(ethptr->pcidev, E1000_PCI_FLASHBASE, (uint32 *)&ethptr->flashbase);

	pci_bios_read_config_dword(ethptr->pcidev, E1000_PCI_MEMBASE, (uint32 *)&ethptr->membase);

//	kprintf("[_82545EMInit]: iobase: %x, flashbase: %x, membase: %x\r\n", ethptr->iobase, ethptr->flashbase, ethptr->membase);
	
	/* Read interrupt line number */
	pci_bios_read_config_byte(ethptr->pcidev, E1000_PCI_IRQ, (uint32 *)&(ethptr->dev->dvirq));
//	kprintf("[_82545EMInit]: interrupt line # %d\r\n", ethptr->dev->dvirq);

	/* Enable PCI bus master, I/O port access */
	pci_bios_read_config_word(ethptr->pcidev, E1000_PCI_COMMAND, &command);
	command |= E1000_PCI_CMD_MASK;
	pci_bios_write_config_word(ethptr->pcidev, E1000_PCI_COMMAND, command);

	/* Read the MAC address */
	/* ToDo: possible bugs 	*/
	rar_low = e1000_io_readl(ethptr->iobase, E1000_RAL(0));
	rar_high = e1000_io_readl(ethptr->iobase, E1000_RAH(0));
	for (i = 0; i<ETH_ADDR_LEN; i++) {
		ethptr->devAddress[i] = (byte)(rar_low >> (i*8) );
	}
	for (i = 0; i<ETH_ADDR_LEN; i++) {
		ethptr->devAddress[i + 4] = (byte)(rar_high >> (i*8) );
	}

	/*kprintf("MAC address is %02x:%02x:%02x:%02x:%02x:%02x\r\n", 
		0xff&(ethptr->devAddress[0]), 
		0xff&(ethptr->devAddress[1]), 
		0xff&(ethptr->devAddress[2]),
		0xff&(ethptr->devAddress[3]), 
		0xff&(ethptr->devAddress[4]), 
		0xff&(ethptr->devAddress[5]));
	*/

	/* Initialize structure pointers */
	ethptr->rxHead = 0;
	ethptr->rxTail = 0;
	ethptr->txHead = 0;
	ethptr->txTail = 0;
	
	ethptr->rxRingSize = E1000_RX_RING_SIZE;
	ethptr->txRingSize = E1000_TX_RING_SIZE;

	ethptr->isem = semcreate(0);
	ethptr->osem = semcreate(E1000_TX_RING_SIZE);

	/* Rings must be aligned on a 16-byte boundary 
	 * lower 2 bits must be zero: 
	 * step1: round up, 16 bytes minimum request
	 * step2: mask of lower 2 bits */
	ethptr->rxRing = (void *)getmem( round16(ethptr->rxRingSize * E1000_RDSIZE) );
	ethptr->txRing = (void *)getmem( round16(ethptr->txRingSize * E1000_TDSIZE) );
	
	if ( (ethptr->rxRing == SYSERR) || (ethptr->txRing == SYSERR) ) {
		kprintf("[82545EMInit]: not enough space for Rings\r\n");
		return;
	}
	ethptr->rxRing = (void *)( ((uint32)ethptr->rxRing + 0xf) & (~0xf) );
	ethptr->txRing = (void *)( ((uint32)ethptr->txRing + 0xf) & (~0xf) );
	
	/* Buffers are highly recommended to be allocated on cache-line */
	/* 	size (64-byte for E8400) 				*/
	ethptr->rxBufs = (void *)getmem( round64(ethptr->rxRingSize * ETH_BUF_SIZE) );
	ethptr->txBufs = (void *)getmem( round64(ethptr->txRingSize * ETH_BUF_SIZE) );

	if ( (ethptr->rxBufs == SYSERR) || (ethptr->txBufs == SYSERR) ) {
		kprintf("[82545EMInit]: not enough space for Bufs\r\n");
		return;
	}
	ethptr->rxBufs = (void *)( ((uint32)ethptr->rxBufs + 0x3f) & (~0x3f) );
	ethptr->txBufs = (void *)getmem( ((uint32)ethptr->txBufs + 0x3f) & (~0x3f) );
	
	//kprintf("[82545EMInit]: rxRing %x, txRing %x, rxBufs %x, txBufs %x\r\n", ethptr->rxRing, ethptr->txRing, ethptr->rxBufs, ethptr->txBufs);

	/* Set buffer pointers and rings to zero */
	memset(ethptr->rxRing, '\0', ethptr->rxRingSize * E1000_RDSIZE);
	memset(ethptr->txRing, '\0', ethptr->txRingSize * E1000_TDSIZE);
	memset(ethptr->rxBufs, '\0', ethptr->rxRingSize * ETH_BUF_SIZE);
	memset(ethptr->txBufs, '\0', ethptr->txRingSize * ETH_BUF_SIZE);

	/* Insert the buffer into descriptor ring 
	 * e.g. rxRing.ptToBuf <- rxBuf.nth
	 * operate on receive && transmit ring accordingly
	 * the last one should point back to the first(no need) */

	rxRingPtr = (struct e1000_rx_desc *)ethptr->rxRing;
	buffer_ptr = (uint64)ethptr->rxBufs; 	
	
	for (i = 0; i < ethptr->rxRingSize; i++) {
		rxRingPtr->buffer_addr = (uint64)buffer_ptr;
		rxRingPtr++;					// since contiguous
		buffer_ptr = buffer_ptr + ETH_BUF_SIZE;
	}

	txRingPtr = (struct e1000_tx_desc *)ethptr->txRing;
	buffer_ptr = (uint64)ethptr->txBufs;
	
	for (i = 0; i < ethptr->txRingSize; i++) {
		txRingPtr->buffer_addr = (uint64)buffer_ptr;
		txRingPtr++;
		buffer_ptr = buffer_ptr + ETH_BUF_SIZE;
	}
	

	/*Reset packet buffer allocation to default */	
	e1000_io_writel(ethptr->iobase, E1000_PBA, E1000_PBA_DEFAULT);	// is default correct?

	/* Reset the NIC to bring it into a known state and initialize it */
	_82545EM_reset_hw(ethptr);

	/* Initialize the hardware */
	if (_82545EM_init_hw(ethptr) != OK)
		return SYSERR;

	/* Configure the NIC */
	e1000_io_writel(ethptr->iobase, E1000_AIT, 0);

	/* Configure the RX */
	_82545EM_configure_rx(ethptr);
	
	/* Configure the TX */
	_82545EM_configure_tx(ethptr);

	/* Register the interrupt and enable interrupt */
	set_evec(ethptr->dev->dvirq + IRQBASE, (uint32)e1000Dispatch);
	e1000IrqEnable(ethptr);

	
	/* debug purpose */
	/* device status */
	uint16 phy_status;
	uint32 ctrl_status, txcw, rxcw;	
	
	_82545EM_read_phy_reg(ethptr, E1000_PHY_STATUS, &phy_status);
	ctrl_status = e1000_io_readl(ethptr->iobase, E1000_STATUS);
	//txcw = e1000_io_readl(ethptr->iobase, E1000_TXCW);
	//rxcw = e1000_io_readl(ethptr->iobase, E1000_RXCW);
	//kprintf("[status register]: PHY_STATUS: %x, STATUS: %x, TXCW: %x, RXCW: %x\r\n", phy_status, ctrl_status, txcw, rxcw);
	
	/* interrupt status */
	uint32 ims, mdic;
	ims = e1000_io_readl(ethptr->iobase, E1000_IMS);
	//imc = e1000_io_readl(ethptr->iobase, E1000_IMC) // invalid!! Write only!!
	mdic = e1000_io_readl(ethptr->iobase, E1000_MDIC);
	//kprintf("[interrupt status]: IMS %8x, MDIC %8x\r\n", ims, mdic);

	return OK;
}

/*------------------------------------------------------------------------
 * _82545EM_reset_hw - Reset the hardware 
 *------------------------------------------------------------------------
 */
local void _82545EM_reset_hw(
	struct 	ether *ethptr
	)
{

	uint32 rctl, tctl, ctrl;
	//uint32 fwsm;

	/* Masking off all interrupts */
	e1000_io_writel(ethptr->iobase, E1000_IMC, 0xffffffff);


	/* Disable the Transmit and Receive units. */
	rctl = e1000_io_readl(ethptr->iobase, E1000_RCTL);
	rctl &= ~E1000_RCTL_EN;
	e1000_io_writel(ethptr->iobase, E1000_RCTL, rctl);

	tctl = e1000_io_readl(ethptr->iobase, E1000_TCTL);
	tctl &= ~E1000_TCTL_EN;
	e1000_io_writel(ethptr->iobase, E1000_TCTL, tctl);

	e1000_io_flush(ethptr->iobase);

	/* Issuing a global reset by setting CTRL register with E1000_CTRL_RST*/
	ctrl = e1000_io_readl(ethptr->iobase, E1000_CTRL);
//	ctrl |= E1000_CTRL_PHY_RST;
	ctrl |= E1000_CTRL_RST;
	e1000_io_writel(ethptr->iobase, E1000_CTRL, ctrl);
	
    	/* Delay slightly to let hardware process */
	DELAY(1);	//delay 1us	

    	/* Masking off all interrupts again*/
	e1000_io_writel(ethptr->iobase, E1000_IMC, 0xffffffff);
}

/*------------------------------------------------------------------------
 * _82545EM_init_hw - Initialize the hardware
 *------------------------------------------------------------------------
 */
local status _82545EM_init_hw(
	struct 	ether *ethptr
	)
{

	uint32 rar_low, rar_high;
	uint32 ctrl, ctrl_ext;
	uint16 phy_ctrl,phy_status, phy_autoneg_adv, phy_1000t_ctrl/*, phy_page_select*/;
	
	int i;

	/* Setup the receive address */
	/* Zero out the other receive addresses */ 
	/* RAL(i) && RAH(i), i<-- 1 to entries */
	rar_low = 0;
	rar_high = 0;
	
	for (i = 1; i< E1000_82545EM_RAR_ENTRIES; i++ ) {
		
		e1000_io_writel(ethptr->iobase, E1000_RAL(i), rar_low);
		e1000_io_flush(ethptr->iobase);
		e1000_io_writel(ethptr->iobase, E1000_RAH(i), rar_high);
		e1000_io_flush(ethptr->iobase);
	}

	/* Zero out the Multicast HASH table */

	for (i = 0; i < E1000_82545EM_MTA_ENTRIES; i++) {
		e1000_io_writel(ethptr->iobase, E1000_MTA + (i<<2), 0);
	}

	/* Configure copper link settings */
	/* Commit the changes.*/
	ctrl = e1000_io_readl(ethptr->iobase, E1000_CTRL);	// follow 14.5.5
	ctrl |= E1000_CTRL_ASDE_EN | E1000_CTRL_SLU | E1000_CTRL_FRCDPX;
	ctrl &= ~E1000_CTRL_FRCSPD ;
	e1000_io_writel(ethptr->iobase, E1000_CTRL, ctrl);
	
	ctrl_ext = e1000_io_readl(ethptr->iobase, E1000_CTRL_EXT);
	ctrl_ext &= ~( (1<<22) | (1<<23) );	// LINK_MODE copper
	ctrl_ext &= ~( (1<<3) | (1<<2) | (1<<1) | (1<<0) );	// GPI_EN
	e1000_io_writel(ethptr->iobase, E1000_CTRL_EXT, ctrl_ext);

	if (OK != _82545EM_read_phy_reg(ethptr, E1000_PHY_CONTROL, &phy_ctrl) ) {
		return SYSERR;
	}

	phy_ctrl |= E1000_MII_CR_RESET;
	
	if (OK != _82545EM_write_phy_reg(ethptr, E1000_PHY_CONTROL, phy_ctrl) ) {
		return SYSERR;
	}

	/* Do a slightly delay for the hardware to proceed the commit */
	DELAY(1);	

	/* Setup autoneg and flow control advertisement and perform 	*/
	/* 	autonegotiation. 					*/
	/* wiki - auto_nego: flow control, duplex, speed 		*/
	/* 0-4: selectro field (802.3)
	 * 5-12: technology ability (100Base-T) (priorities)
	 * 13: remote fault
	 * 14: acknoledgement
	 * 15: next page						*/

	if (OK != _82545EM_read_phy_reg(ethptr, E1000_PHY_AUTONEG_ADV, &phy_autoneg_adv)) {
		return SYSERR;
	}
	if (OK != _82545EM_read_phy_reg(ethptr, E1000_PHY_1000T_CTRL, &phy_1000t_ctrl) ) {
		return SYSERR;
	}
	phy_autoneg_adv |= E1000_NWAY_AR_10T_HD_CAPS | E1000_NWAY_AR_10T_FD_CAPS | 
				E1000_NWAY_AR_100TX_HD_CAPS | E1000_NWAY_AR_100TX_FD_CAPS;

	phy_1000t_ctrl	|= E1000_CR_1000T_HD_CAPS | E1000_CR_1000T_FD_CAPS;
	
	phy_autoneg_adv |= E1000_NWAY_AR_PAUSE | E1000_NWAY_AR_ASM_DIR ;
	
	if (OK != _82545EM_write_phy_reg(ethptr, E1000_PHY_AUTONEG_ADV, phy_autoneg_adv) ) {
		return SYSERR;
	}
	if (OK != _82545EM_write_phy_reg(ethptr, E1000_PHY_1000T_CTRL, phy_1000t_ctrl) ) {
		return SYSERR;
	}

	/* Restart auto-negotiation. */
	if ( OK != _82545EM_read_phy_reg(ethptr, E1000_PHY_CONTROL, &phy_ctrl) ) {
		return SYSERR;
	}
	phy_ctrl |= E1000_MII_CR_RESTART_AUTO_NEG | E1000_MII_CR_AUTO_NEG_EN;

	if ( OK != _82545EM_write_phy_reg(ethptr, E1000_PHY_CONTROL, phy_ctrl) ) {
		return SYSERR;
	}
	/* Wait for auto-negotiation to complete 
       Implement a loop here to check the E1000_MII_SR_LINK_STATUS and E1000_MII_SR_AUTONEG_COMPLETE, break if they are both ture
       You should also delay for a while in each loop so it won't take too much CPU time */
	if ( OK != _82545EM_read_phy_reg(ethptr, E1000_PHY_STATUS, &phy_status) ) {
		return SYSERR;
	}

	while(1) {
		if (phy_status & E1000_MII_SR_LINK_STATUS) {
			if (phy_status & E1000_MII_SR_AUTONEG_COMPLETE) {
				break;
			}
		}

		MDELAY(100);
		
		if (OK != _82545EM_read_phy_reg(ethptr, E1000_PHY_STATUS, &phy_status)) {
			return SYSERR;
		}
	}
    	
	
	/* Update device control according receive flow control and transmit flow control*/
	ctrl = e1000_io_readl(ethptr->iobase, E1000_CTRL);
	ctrl &= ( ~(E1000_CTRL_TFCE | E1000_CTRL_RFCE) );
	e1000_io_writel(ethptr->iobase, E1000_CTRL, ctrl);


	return OK;
}

/*------------------------------------------------------------------------
 * _82545EM_configure_rx - Configure Receive Unit after Reset
 *------------------------------------------------------------------------
 */
local void _82545EM_configure_rx(
	struct 	ether *ethptr
	)
{

	uint32	rctl, rxcsum;
	/* Disable receiver while configuring. */
	rctl = e1000_io_readl(ethptr->iobase, E1000_RCTL);
	rctl &= ~E1000_RCTL_EN;
	e1000_io_writel(ethptr->iobase, E1000_RCTL, rctl);

	/* Enable receiver, accept broadcast packets, no loopback, and 	*/
	/* 	free buffer threshold is set to 1/2 RDLEN. 		*/

	rctl |= E1000_RCTL_EN | E1000_RCTL_BAM | E1000_RCTL_LBM_NO | E1000_RCTL_RDMTS_HALF ;


	/* Do not store bad packets, do not pass MAC control frame, 	*/
	/* 	disable long packet receive and CRC strip 		*/
	rctl &= ~( E1000_RCTL_SBP | E1000_RCTL_PMCF | E1000_RCTL_LPE | E1000_RCTL_SECRC );
	
	/* Setup buffer sizes */
	// CHECK!!!
	rctl |= E1000_RCTL_SZ_2048; // align with ETH_BUF	

	/* Set the Receive Delay Timer Register, let driver be notified */
	/* 	immediately each time a new packet has been stored in 	*/
	/* 	memory 							*/
	e1000_io_writel(ethptr->iobase, E1000_RDTR, E1000_RDTR_DEFAULT); // zero delay

	/* Set up interrupt rate to be default. Notice that it is a the rate is not just E1000_ITR_DEFAULT which is the frequency, 
       it is 1000000000 / (E1000_ITR_DEFAULT * 256) */
	e1000_io_writel(ethptr->iobase, E1000_ITR, 1000000000/(E1000_ITR_DEFAULT * 256) );


	/* Setup the HW Rx Head and Tail Descriptor Pointers, the Base 	*/
	/* 	and Length of the Rx Descriptor Ring 			*/

	e1000_io_writel(ethptr->iobase, E1000_RDH(0), 0);
	e1000_io_writel(ethptr->iobase, E1000_RDT(0), ethptr->rxRingSize - E1000_RING_BOUNDARY);
	e1000_io_writel(ethptr->iobase, E1000_RDBAL(0), (uint32)ethptr->rxRing );
	e1000_io_writel(ethptr->iobase, E1000_RDBAH(0), 0 );	// 32-bit addr.
	e1000_io_writel(ethptr->iobase, E1000_RDLEN(0), ethptr->rxRingSize * E1000_RDSIZE ); // round16() ?


	/* Disable Receive Checksum Offload for IPv4, TCP and UDP. */
	rxcsum = e1000_io_readl(ethptr->iobase, E1000_RXCSUM);
	rxcsum &= ~( E1000_RXCSUM_IPOFL | E1000_RXCSUM_TUOFL );
	e1000_io_writel(ethptr->iobase, E1000_RXCSUM, rxcsum );

	/* Enable receiver. */
	e1000_io_writel(ethptr->iobase, E1000_RCTL, rctl );
	DELAY(1);	// make sure 
}

/*------------------------------------------------------------------------
 * _82545EM_configure_tx - Configure Transmit Unit after Reset
 *------------------------------------------------------------------------
 */
local void _82545EM_configure_tx(
	struct 	ether *ethptr
	)
{
	uint32	txdctl, tctl, tipg;
	uint32	temp1, temp2;

	/* Set the transmit descriptor write-back policy for both queues */
	/* ??? Write-back threshold ??? */
	txdctl = e1000_io_readl(ethptr->iobase, E1000_TXDCTL(0) );
	txdctl |= E1000_TXDCTL_WTHRESH | E1000_TXDCTL_GRAN;
	e1000_io_writel(ethptr->iobase, E1000_TXDCTL(0), txdctl );

	/* Program the Transmit Control Register */
	/* refer chap 14.5 in manual */
	tctl = e1000_io_readl(ethptr->iobase, E1000_TCTL);
	tctl |= E1000_TCTL_EN | E1000_TCTL_PSP | E1000_TCTL_CT | E1000_TCTL_COLD;

	/* Set the default values for the Tx Inter Packet Gap timer */
	temp1 = E1000_TIPG_IPGR1_DEFAULT;
	temp1 = temp1 << E1000_TIPG_IPGR1_SHIFT;
	temp2 = E1000_TIPG_IPGR2_DEFAULT;
	temp2 = temp2 << E1000_TIPG_IPGR2_SHIFT;
	tipg = E1000_TIPG_IPGT_COPPER_DEFAULT | temp1 | temp2;
	e1000_io_writel(ethptr->iobase, E1000_TIPG, tipg);

	/* Set the Tx Interrupt Delay register */
	e1000_io_writel(ethptr->iobase, E1000_TIDV, E1000_TIDV_DEFAULT );
	e1000_io_writel(ethptr->iobase, E1000_TADV, E1000_TADV_DEFAULT );

	/* Setup the HW Tx Head and Tail descriptor pointers */
	e1000_io_writel(ethptr->iobase, E1000_TDH(0), 0);
	e1000_io_writel(ethptr->iobase, E1000_TDT(0), 0);
	e1000_io_writel(ethptr->iobase, E1000_TDLEN(0), E1000_TDSIZE * ethptr->txRingSize);
	e1000_io_writel(ethptr->iobase, E1000_TDBAL(0), (uint32)ethptr->txRing );
	e1000_io_writel(ethptr->iobase, E1000_TDBAH(0), 0);

	/* Enable transmit but setting TCTL*/
	e1000_io_writel(ethptr->iobase, E1000_TCTL, tctl);
	DELAY(1);
}

/*------------------------------------------------------------------------
 * _82545EM_read_phy_reg - Read MDI control register
 *------------------------------------------------------------------------
 */
status _82545EM_read_phy_reg(
	struct 	ether *ethptr,
	uint32 	offset,
	uint16 	*data
	)
{
	uint32 i, mdic = 0;

	if (offset > E1000_MAX_PHY_REG_ADDRESS) {
		return SYSERR;
	}

	mdic = ((offset << E1000_MDIC_REG_SHIFT) |
		(E1000_82545EM_MDIC_PHY_ADDR << E1000_MDIC_PHY_SHIFT) |
		(E1000_MDIC_OP_READ));

	e1000_io_writel(ethptr->iobase, E1000_MDIC, mdic);

	for (i = 0; i < (E1000_GEN_POLL_TIMEOUT * 3); i++) {
		DELAY(50);
		mdic = e1000_io_readl(ethptr->iobase, E1000_MDIC);
		if (mdic & E1000_MDIC_READY)
			break;
	}
	if (!(mdic & E1000_MDIC_READY)) {
		return SYSERR;
	}
	if (mdic & E1000_MDIC_ERROR) {
		return SYSERR;
	}
	*data = (uint16) mdic;

	return OK;
}

/*------------------------------------------------------------------------
 *  _82545EM_write_phy_reg - Write MDI control register
 *------------------------------------------------------------------------
 */
status _82545EM_write_phy_reg(
	struct 	ether *ethptr,
	uint32 	offset,
	uint16 	data
	)
{
	uint32 i, mdic = 0;

	if (offset > E1000_MAX_PHY_REG_ADDRESS) {
		return SYSERR;
	}

	mdic = ( ((uint32)data) |
		 (offset << E1000_MDIC_REG_SHIFT) |
		 (E1000_82545EM_MDIC_PHY_ADDR << E1000_MDIC_PHY_SHIFT) |
		 (E1000_MDIC_OP_WRITE) );

	e1000_io_writel(ethptr->iobase, E1000_MDIC, mdic);

	for (i = 0; i < (E1000_GEN_POLL_TIMEOUT * 3); i++) {
		DELAY(50);
		mdic = e1000_io_readl(ethptr->iobase, E1000_MDIC);
		if (mdic & E1000_MDIC_READY)
			break;
	}
	if (!(mdic & E1000_MDIC_READY)) {
		return SYSERR;
	}
	if (mdic & E1000_MDIC_ERROR) {
		return SYSERR;
	}

	return OK;
}


