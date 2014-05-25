/* ethInit.c - ethInit */

#include <xinu.h>
#define DEBUG	0

struct	ether	ethertab[Neth];		/* Ethernet control blocks 	*/
/* Neth == 1, defined in conf.h */

/*------------------------------------------------------------------------
 * ethInit - Initialize Ethernet device structures
 *------------------------------------------------------------------------
 */
devcall	ethInit (
	  struct dentry *devptr
	)
{
	struct	ether 	*ethptr;
	int32	dinfo;			/* device information*/

	/* Initialize structure pointers */
	ethptr = &ethertab[devptr->dvminor];
	
	memset(ethptr, '\0', sizeof(struct ether));	/* clear ether entry */

	/* find Intel 82545EM */
	/* If found, intialize the devtab of ETHER0 with function pt of e10000 */
	/* If !found, return SYSERR*/
	
	dinfo = find_pci_device( INTEL_82545EM_DEVICE_ID, INTEL_VENDOR_ID, 0);

	if ( dinfo != SYSERR ) {
		// kprintf("[ethInit]: Found Intel82545EM! base: %x\r\n", dinfo);
		// see checklist, make sure init every field of ethptr!! 
		
		ethptr->state = ETH_STATE_DOWN;
		ethptr->type = NIC_TYPE_82545EM;

		ethptr->dev = devptr;
		ethptr->csr = devptr->dvcsr;
		ethptr->pcidev = dinfo;
		// iobase, flashbase, membase
		// rxRing, rxBufs, rxHead, rxTail, rxRingSize, rxIrq
		// txRing, txBufs, txHead, txTail, txRingSize, txIrq
		// devAddress

		ethptr->addrLen = ETH_ADDR_LEN;
		ethptr->mtu = ETH_MTU;
		
		ethptr->errors = 0;
	
		// isem, osem, istart
		// inPool, outPool, proms
		// ed_mcset, ed_mcc, ed_mca

		//ethptr->rxHead = ethptr->rxTail = 0;
		//ethptr->txHead = ethptr->txTail = 0;


		/* device specific function pointer */
		ethptr->ethInit = _82545EMInit;
		ethptr->ethRead = e1000Read;
		ethptr->ethWrite = e1000Write;
		ethptr->ethControl = e1000Control;
		ethptr->ethInterrupt = e1000Interrupt;
	}
	else {
		kprintf("[ethInit]: Didn't find Intel82545EM! \r\n");
		return SYSERR;
	}
	
	ethptr->ethInit(ethptr);
	/* Set the state of ethptr to be UP */
	ethptr->state = ETH_STATE_UP;
	
	return OK;
}
