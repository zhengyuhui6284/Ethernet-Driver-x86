/* e1000Write.c - e1000Write */

#include <xinu.h>

/*------------------------------------------------------------------------
 * e1000Write - write a packet to an E1000 device
 *------------------------------------------------------------------------
 */
devcall	e1000Write(
	struct	dentry	*devptr,	/* entry in device switch table	*/
	void	*buf,			/* buffer that holds a packet	*/
	uint32	len			/* length of buffer		*/
	)
{
	struct	ether *ethptr;
	struct 	e1000_tx_desc *descptr;/* ptr to ring descriptor 	*/
	char 	*pktptr; 		/* ptr used during packet copy  */
	uint32	tail;			/* index of ring buffer for pkt	*/
	uint32 	tdt;

	uint32 	tctl;			/* transmit control register	*/
	/* verify ether is UP & arguments are valid */
	ethptr = &ethertab[devptr->dvminor];
	
	/*if ( ethptr->state != ETH_STATE_UP ) 
		kprintf("[e1000Write]: !ETH_STATE_UP\r\n");
		return SYSERR;
	*/
	/* If short packet is enabled, and if len <  17 	*/
	/* return error						*/
	tctl = e1000_io_readl(ethptr->iobase, E1000_TCTL);
	if ( tctl & E1000_TCTL_PSP ) {		/* padding is enabled */
		if (len < 17)
			return SYSERR;
	}

//kprintf("[e1000Write]: ethptr->osem %d\r\n", ethptr->osem);
	/* free ring slot? */
	wait(ethptr->osem);
//kprintf("[e1000Write]: got osem\r\n");
	/* Find the tail of the ring to insert packet */
	tail = ethptr->txTail;			/* tail offset	*/
	descptr = (struct e1000_tx_desc *)ethptr->txRing + tail;

	/* Copy to transmit ring buffer (user space -> kernel space) */
	pktptr = (char *)((uint32)descptr->buffer_addr & ADDR_BIT_MASK);
	memcpy(pktptr, buf, len);
//kprintf("[e1000Write]: passed here!\r\n");
	/* Insert transmitting command and length */
	descptr->lower.data &= E1000_TXD_CMD_DEXT;
	descptr->lower.data = E1000_TXD_CMD_IDE |
			      E1000_TXD_CMD_RS | 
			      E1000_TXD_CMD_IFCS |
			      E1000_TXD_CMD_EOP |
			      len;
	descptr->upper.data = 0;

	/* Add descriptor by advancing the tail pointer (lower)		*/	
	tdt = e1000_io_readl(ethptr->iobase, E1000_TDT(0));
	tdt = (tdt + 1) % ethptr->txRingSize;
	e1000_io_writel(ethptr->iobase, E1000_TDT(0), tdt);

	/* ring tail pointing to the next ring descriptor (upper) 	*/
	ethptr->txTail = (ethptr->txTail + 1) % ethptr->txRingSize;

//	kprintf("[e1000Write]: len %d\r\n",len);
	return len;
}
