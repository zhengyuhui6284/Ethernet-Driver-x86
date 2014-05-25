/* e1000Read.c - e1000Read */

#include <xinu.h>

/*------------------------------------------------------------------------
 * e1000Read - read a packet from an E1000 device
 *------------------------------------------------------------------------
 */
devcall	e1000Read(
	struct	dentry *devptr,		/* entry in device switch table	*/
	void	*buf,			/* buffer to hold packet	*/
	uint32	len			/* length of buffer		*/
	)
{

	struct	ether *ethptr;
	struct	e1000_rx_desc *descptr;/* ptr to ring descriptor	*/
	char	*pktptr;		/* ptr used during packet copy	*/
	uint32	head;			/* head of ring buffer 		*/
	uint32	status;			/* status of entry		*/
	uint32	length;			/* packet length		*/
	int32 	retval;
	uint32 	rdt;

	ethptr = &ethertab[devptr->dvminor];

//kprintf("[e1000Read]: flag1 - isem %d\r\n", ethptr->isem);
	/* Wait for a packet to arrive */
	wait(ethptr->isem);
//kprintf("[e1000Read]: flag2\r\n");
	/* Find out where to pick up the packet (upper) */
	head = ethptr->rxHead;
	descptr = (struct e1000_rx_desc *)ethptr->rxRing + head;
	
	pktptr = (char *)((uint32)(descptr->buffer_addr & ADDR_BIT_MASK));
	length = descptr->length;
	memcpy(buf, pktptr, length);
	retval = length;

//kprintf("[e1000Read]: %s \r\n", (char*)buf);
	
	/* Clear up the descriptor and the buffer */
	descptr->length = 0;
	descptr->csum = 0;
	descptr->status = 0;
	descptr->errors = 0;
	descptr->special = 0;
	memset((char *)((uint32)(descptr->buffer_addr & ADDR_BIT_MASK)), '\0', ETH_BUF_SIZE); 

	/* Add newly reclaimed descriptor to the ring (lower) */
	if (ethptr->rxHead % E1000_RING_BOUNDARY == 0) {
		rdt = e1000_io_readl(ethptr->iobase, E1000_RDT(0));
		rdt = (rdt + E1000_RING_BOUNDARY) % ethptr->rxRingSize;
		e1000_io_writel(ethptr->iobase, E1000_RDT(0), rdt);
	}

	/* head pointing to the next ring descriptor (upper) */
	ethptr->rxHead = (ethptr->rxHead + 1) % ethptr->rxRingSize;

//	kprintf("[e1000Read]: retval %d\r\n", retval);
	return retval;
}
