#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/timer.h>

#include <plat/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/map.h>

#include "test_mcbsp.h"

typedef unsigned int	U32;
typedef unsigned short	U16;
typedef unsigned char	U8;
void out_regl (u32 addr, u32 val)
{
	omap_writel (val, addr);
}

u32 in_regl (u32 addr)
{
	return omap_readl (addr);
}

void mcbsp_clk_init(U32 sid)
{
    switch(sid)
    {
        /* McBSP 1*/
        case 1: 
            /* Enable the functional clock of McBSP1 */
            out_regl(CM_FCLKEN1_CORE, (in_regl(CM_FCLKEN1_CORE) | (0x1 << 9)));
            /* Enable the Interface clock of McBSP1 */
            out_regl(CM_ICLKEN1_CORE, (in_regl(CM_ICLKEN1_CORE) | (0x1 << 9)));
            out_regl(CM_AUTOIDLE1_CORE, (in_regl(CM_AUTOIDLE1_CORE) | (0x1 << 9)));
            /*configure the FSR & CLR */
            out_regl(CONTROL_DEVCONF0, (in_regl(CONTROL_DEVCONF0) | ((0x1 << 3) | (0x1 << 4))));
            break;

        /* McBSP 2*/
        case 2:
            /* Enable the functional clock of McBSP2 */
            out_regl(CM_FCLKEN_PER, (in_regl(CM_FCLKEN_PER) | 0x1));
            /* Enable the Interface clock of McBSP2 */
            out_regl(CM_FCLKEN_PER, (in_regl(CM_FCLKEN_PER) | 0x1));
            out_regl(CM_AUTOIDLE_PER, (in_regl(CM_AUTOIDLE_PER) | 0x1));
            /* Reset Bit 6 to configure the CLKS input to 96M functional clk from PRCM */
            out_regl(CONTROL_DEVCONF0, (in_regl(CONTROL_DEVCONF0) & 0xFFFFFFBF));
            break;
	
        /* McBSP 3*/
        case 3:
			
            /* Enable the functional clock of McBSP3 */
            out_regl(CM_FCLKEN_PER, (in_regl(CM_FCLKEN_PER) | 0x2));
            /* Enable the Interface clock of McBSP3 */
            out_regl(CM_ICLKEN_PER, (in_regl(CM_ICLKEN_PER) | 0x2));
            out_regl(CM_AUTOIDLE_PER, (in_regl(CM_AUTOIDLE_PER) | 0x2));
            /* Reset Bit 0 to configure the CLKS input to 96M functional clk from PRCM */
            out_regl(CONTROL_DEVCONF1, (in_regl(CONTROL_DEVCONF1) & 0xFFFFFFFE));
            break;
        /* McBSP 4*/
        case 4:
		
            /* Enable the functional clock of McBSP4 */
            out_regl(CM_FCLKEN_PER, (in_regl(CM_FCLKEN_PER) | 0x4));
            /* Enable the Interface clock of McBSP4 */
            out_regl(CM_ICLKEN_PER, (in_regl(CM_ICLKEN_PER) | 0x4));
            out_regl(CM_AUTOIDLE_PER, (in_regl(CM_AUTOIDLE_PER) | 0x4));
            /* Reset Bit 3-0 to configure the CLKS input to 96M functional clk from PRCM */
            out_regl(CONTROL_DEVCONF1, (in_regl(CONTROL_DEVCONF1) & 0xFFFFFFFB));
            break;
	}
}

void enable_mcbsp_clks(U32 sid)
{
	mcbsp_clk_init(sid);
}

/*-----------------------------------------------------------------------------
| Function    : reset_mcbsp()
+------------------------------------------------------------------------------
| Description : reset the mcbsp
|
| Parameters  : <description of in/out parameters>
|
| Returns     : status_code (ok or failed)
+-----------------------------------------------------------------------------*/
void reset_mcbsp(U32 mcbsp_base_addr ,U32 enable)
{
    /* Checking whether McBSP is providing the clock (Word and bit CLK) or McBSP is master  */
    //if( ((*(volatile U32 *)(mcbsp_base_addr + MCBSP_PCR0_OFFSET)) & 0x0F00) != 0x0F00)
    if( (in_regl(mcbsp_base_addr + MCBSP_PCR0_OFFSET) & 0x0F00) != 0x0F00)
    {
        /* If McBSP is not master, then TX/RX/SCG are not enabled and disabled between
        each call to the macbsp_write/read functions. This is a workaround.
        Need to remove this once the driver is improved. */
        return;
    }
    else
    {
        //*((volatile U32 *)(mcbsp_base_addr + MCBSP_DXR_OFFSET))= 0x0;
        out_regl((mcbsp_base_addr + MCBSP_DXR_OFFSET), 0x0);
        if (enable)
        {
            // The serial port transmitter is enabled.
            //*((volatile U32 *)(mcbsp_base_addr + MCBSP_SPCR2_OFFSET)) |= 0x0001;
			out_regl((mcbsp_base_addr + MCBSP_SPCR2_OFFSET), (in_regl(mcbsp_base_addr + MCBSP_SPCR2_OFFSET) | 0x0001));


            // The serial port receiver is enabled.
            //*((volatile U32 *)(mcbsp_base_addr + MCBSP_SPCR1_OFFSET)) |= 0x0001;
			out_regl((mcbsp_base_addr + MCBSP_SPCR1_OFFSET), (in_regl(mcbsp_base_addr + MCBSP_SPCR1_OFFSET) | 0x0001));

            // Sample rate generator is pulled out of reset.
            // frame counters are loaded with their programmed values.
            //*((volatile U32 *)(mcbsp_base_addr + MCBSP_SPCR2_OFFSET)) |= 0x00C0;  // bit 6, 7
			out_regl((mcbsp_base_addr + MCBSP_SPCR2_OFFSET), (in_regl(mcbsp_base_addr + MCBSP_SPCR2_OFFSET) | 0x00C0));
        }
        else
        {
            // The serial port transmitter is disabled and in reset state.
            // Frame-synchronization logic is reset.
            //*((volatile U32 *)(mcbsp_base_addr + MCBSP_SPCR2_OFFSET)) &= ~(0x00C1);
			out_regl((mcbsp_base_addr + MCBSP_SPCR2_OFFSET), (in_regl(mcbsp_base_addr + MCBSP_SPCR2_OFFSET) & ~(0x00C1)));

            // The serial port receiver is disabled and in reset state.
            //*((volatile U32 *)(mcbsp_base_addr + MCBSP_SPCR1_OFFSET)) &= ~(0x0001);
			out_regl((mcbsp_base_addr + MCBSP_SPCR1_OFFSET), (in_regl(mcbsp_base_addr + MCBSP_SPCR1_OFFSET) & ~(0x0001)));
        }
    }
}

/*-----------------------------------------------------------------------------
| Function    : config_mcbsp()
+------------------------------------------------------------------------------
| Description : config_mcbsp_master
|
| Parameters  : <description of in/out parameters>
|
| Returns     : status_code (ok or failed)
+-----------------------------------------------------------------------------*/
U8 config_mcbsp(U32 mcbsp_base_addr,
                U8 master,
                U32 clksr_val,						/* sampling rate value/ bit clk value	*/
                U8 mode,
				U32 sid
               )
{

    U16 srgdiv= 53;
    U8 tx_data_delay = 1;
    U8 rx_data_delay = 1;
	U8 clk_xp = 1;
	U8 clk_rp = 1;

	/* Determine which clock to be enabled */
    enable_mcbsp_clks(sid);
	mdelay (100);
    /* make all registers to zero to start with */
    out_regl((mcbsp_base_addr + MCBSP_SPCR1_OFFSET), 0);
    out_regl((mcbsp_base_addr + MCBSP_SPCR2_OFFSET), 0);
    out_regl((mcbsp_base_addr + MCBSP_PCR0_OFFSET), 0);
    out_regl((mcbsp_base_addr + MCBSP_XCR1_OFFSET), 0);
    out_regl((mcbsp_base_addr + MCBSP_RCR1_OFFSET), 0);
    out_regl((mcbsp_base_addr + MCBSP_XCR2_OFFSET), 0);
    out_regl((mcbsp_base_addr + MCBSP_RCR2_OFFSET), 0);
    out_regl((mcbsp_base_addr + MCBSP_SRGR1_OFFSET), 0);
    out_regl((mcbsp_base_addr + MCBSP_SRGR2_OFFSET), 0);
    /* disable tx and rx dma */
    out_regl((mcbsp_base_addr + MCBSP_XCCR_OFFSET), 0);
    out_regl((mcbsp_base_addr + MCBSP_RCCR_OFFSET), 0);
    out_regl((mcbsp_base_addr + MCBSP_THRSH2_OFFSET), 0);
    out_regl((mcbsp_base_addr + MCBSP_THRSH1_OFFSET), 0);
    if(master)
    {
        clk_xp = 0; /* 0 -> Raising Edge , 1 Falling Edge */
		tx_data_delay = 1;
		out_regl((mcbsp_base_addr + MCBSP_PCR0_OFFSET),0x0f00 | (clk_xp << 1) | clk_rp);
		out_regl((mcbsp_base_addr + MCBSP_SPCR1_OFFSET),
		(in_regl((mcbsp_base_addr + MCBSP_SPCR1_OFFSET)) | 0x0008));
		out_regl((mcbsp_base_addr + MCBSP_SPCR2_OFFSET),
		(in_regl((mcbsp_base_addr + MCBSP_SPCR2_OFFSET)) | 0x0008));
        /* Tx word len1= 16bits,1 word per frame */
#if 1
		out_regl((mcbsp_base_addr + MCBSP_XCR1_OFFSET),
		(in_regl((mcbsp_base_addr + MCBSP_XCR1_OFFSET)) | (2 << 5) | (128 << 8)));
        /* 	Dual phase frame
                      *	Tx word len2=16 bits ,1 word per frame.
                      *	No companding data starts with MSB first
                      *	Transmit frame-synchronization pulses after the first are ignored.
                      *	Tx Data delay- data_delay.
                      */
		out_regl((mcbsp_base_addr + MCBSP_XCR2_OFFSET),
		(in_regl((mcbsp_base_addr + MCBSP_XCR2_OFFSET)) | (0x0040 | (tx_data_delay & 0x3))));
        /* Rx word len1= 16bits,1 word per frame */
		out_regl((mcbsp_base_addr + MCBSP_RCR1_OFFSET),
		(in_regl((mcbsp_base_addr + MCBSP_RCR1_OFFSET)) | (2 << 5) | (128 << 8)));
        /* 	Dual phase frame
                      *	Rx word len2=16 bits ,1 word per frame.
                      *	No companding data starts with MSB first
                      *	Rx Data delay- data_delay.
                      */
		out_regl((mcbsp_base_addr + MCBSP_RCR2_OFFSET),
	      (in_regl((mcbsp_base_addr + MCBSP_RCR2_OFFSET)) | (0x0040 | (rx_data_delay & 0x3))));
#endif
        /* Configure the sample generator */
        switch (clksr_val)
        {
		    case 48000:
                srgdiv = 53;
                break;
            case 44100:
                srgdiv = 69;
                break;
            case 22000:
                srgdiv = 135;
                break;
            case 11000:
                srgdiv = 271;
                break;
            case 8000:
                srgdiv = 374;
                break;
        }
		out_regl((mcbsp_base_addr + MCBSP_SRGR1_OFFSET),((16 - 1) << 8) | srgdiv);
        /* Next frame-sync signal becomes active after 32 clock periods.*/
		out_regl((mcbsp_base_addr + MCBSP_SRGR2_OFFSET),
		(in_regl((mcbsp_base_addr + MCBSP_SRGR2_OFFSET)) | ((0x1 << 13) | (0x1 << 12) | ((16 * 2) - 1))));
		//out_regl ((mcbsp_base_addr + MCBSP_THRSH2_OFFSET), 0x7E);
		out_regl ((mcbsp_base_addr + MCBSP_THRSH1_OFFSET), 0x7E);
        /* Enable the sample rate generator */
		out_regl((mcbsp_base_addr + MCBSP_SPCR2_OFFSET),
		(in_regl((mcbsp_base_addr + MCBSP_SPCR2_OFFSET)) | (1 << 6)));
		udelay (1000);
        /* Enable the Tx, Rx */
		out_regl((mcbsp_base_addr + MCBSP_SPCR1_OFFSET),
		(in_regl((mcbsp_base_addr + MCBSP_SPCR1_OFFSET)) | 0x1));
		out_regl((mcbsp_base_addr + MCBSP_SPCR2_OFFSET),
		(in_regl((mcbsp_base_addr + MCBSP_SPCR2_OFFSET)) | 0x0));
        udelay (1000);

    }
    /*==================================================
    McBSP As Slave . CODEC as master.
    ===================================================*/
	
    else	/* McBSP as Slave. */
    {
        /* Rx word len1= 16bits,1 word per frame */
		out_regl((mcbsp_base_addr + MCBSP_RCR1_OFFSET),0x40);
		/* Transmit control register 1 and 2 */
        /* 1 word per frame, 16 bits per word - bits#7:5 = 0x10 */
		out_regl((mcbsp_base_addr + MCBSP_XCR1_OFFSET),0x0040); 

        /* 
        *	Rx word len2=16 bits ,1 word per frame.
        *	No companding data starts with MSB first
        *	Rx Data delay- data_delay.
		*    Tx Data delay- 1 bit.
        */
		if(mode == 0)   /* Dual Phase frame */
		{
			out_regl((mcbsp_base_addr + MCBSP_RCR2_OFFSET),(0x8040 | (rx_data_delay & 0x3)));
        	out_regl((mcbsp_base_addr + MCBSP_XCR2_OFFSET),(0x8040 | (tx_data_delay & 0x3)));
        }
		else  						 /* Single Phase frame */
		{
            /* PCM (For T2 Mode 1)::  Rx on Falling Edge , Tx on Raising Edge , Tx / Rx delay = 0*/
            rx_data_delay = tx_data_delay = clk_xp = clk_rp = 0;
            out_regl((mcbsp_base_addr + MCBSP_RCR2_OFFSET),(0x0040 | (rx_data_delay & 0x3)));
        	out_regl((mcbsp_base_addr + MCBSP_XCR2_OFFSET),(0x0040 | (tx_data_delay & 0x3)));

		}

        
        /* Pin control register */
		/*
        CLKXM-Transmitter clock is driven by an external clock with CLKX as an input pin.
        FSRM- Frame-synchronization pulses generated by an external device. FSR is an input pin.
        Rx and Tx frame-synchronization Polarity : active low
        Tx clock polarity :Tx data send on falling edge of CLKX
        Receive clock polarity :Receive data sampled on rising edge of CLKR
		*/
		out_regl((mcbsp_base_addr + MCBSP_PCR0_OFFSET),(0x008C | clk_xp << 1 | clk_rp));
        /* Sample Rate Gererator Register 1 and 2 */
        /* Sample rate generator clock derived from the McBSPn_FCLK clock. */
		out_regl((mcbsp_base_addr + MCBSP_SRGR2_OFFSET),0x2000);
		out_regl((mcbsp_base_addr + MCBSP_SRGR1_OFFSET),0); 
        /* clear the Tx data register */
		out_regl((mcbsp_base_addr + MCBSP_DXR_OFFSET),0); 
        /*
        The following line enables XRST.
        Tx reset. This resets and enables the Tx.
        */
		out_regl((mcbsp_base_addr + MCBSP_SPCR2_OFFSET), (in_regl((mcbsp_base_addr + MCBSP_SPCR2_OFFSET)) | 0x0001));
        /*
        The following line enables RRST.
        Receiver reset. This resets and enables the receiver.
        */
		out_regl((mcbsp_base_addr + MCBSP_SPCR1_OFFSET), (in_regl((mcbsp_base_addr + MCBSP_SPCR1_OFFSET)) | 0x0001));
    }    return 0;
}


/*-----------------------------------------------------------------------------
| Function    : read_data_mcbsp()
+------------------------------------------------------------------------------
| Description : Reads the RX data register by polling method.
|
| Parameters  : McBSP base addr
|
|
|
| Returns     : status_code (ok or failed)
+-----------------------------------------------------------------------------*/
static int read_cnt = 0;
s32 read_data_mcbsp( U32 mcbsp_base_addr,
                   U16 *read_buffer
                  )
{

    U32 ret_status,attempts;
    // if frame sync error - clear the error
    if((in_regl(mcbsp_base_addr + MCBSP_SPCR1_OFFSET)) & 0x08)
    {
        // clear error
		out_regl ((mcbsp_base_addr + MCBSP_SPCR1_OFFSET), ((in_regl(mcbsp_base_addr + MCBSP_SPCR1_OFFSET)) & ~(0x08)));
		printk ("Recieve frame sync error\n");

    }
    {
        // wait for receive confirmation
        attempts = 0;
		ret_status = 0;
        while (!(ret_status & 0x2))
        {
		udelay(100);
			ret_status = in_regl(mcbsp_base_addr + MCBSP_SPCR1_OFFSET);
			//printk ("Reciver is not ready\n");
#if 1
    		if (attempts++ >1000)
			{
				printk ("mcbsp read error\n");
                		return -1;
			}
#endif
        }
    }
	*read_buffer = in_regl(mcbsp_base_addr + MCBSP_DRR_OFFSET);
	//printk ("Data read:0x%x\n", *read_buffer);
	read_cnt++;

    ret_status = 0;	/* success */
    return ret_status;

}

/*-----------------------------------------------------------------------------
| Function    : write_data_mcbsp()
+------------------------------------------------------------------------------
| Description : writes TX data register by polling method.
|
| Parameters  : McBSP base addr
|								pointer to data variable holding the the data to be written.
|								Data should be a short
|
| Returns     : status_code (ok or failed)
+-----------------------------------------------------------------------------*/
s32 write_data_mcbsp( U32 mcbsp_base_addr,
                    U16 *write_buffer
                   )
{
    U32 ret_status,attempts;

	out_regl ((mcbsp_base_addr + MCBSP_DXR_OFFSET), *write_buffer);

    // if frame sync error - clear the error
	ret_status = in_regl ((mcbsp_base_addr + MCBSP_SPCR2_OFFSET));
    if(ret_status & 0x08)
    {
        // clear error
		out_regl ((mcbsp_base_addr + MCBSP_SPCR2_OFFSET),(in_regl(mcbsp_base_addr + MCBSP_SPCR2_OFFSET) & ~(0x08))); 

    }

    {
        // wait for transmit confirmation
        attempts = 0;
        while (((in_regl(mcbsp_base_addr + MCBSP_SPCR2_OFFSET)) & 0x2) != 0x2)
        {
            if (attempts++ >1000)
            {
                // The attempt failed ,so reset the Tx
				out_regl((mcbsp_base_addr + MCBSP_SPCR2_OFFSET), ((in_regl(mcbsp_base_addr + MCBSP_SPCR2_OFFSET) & ~(0x1))));

                // enable Tx
				out_regl((mcbsp_base_addr + MCBSP_SPCR2_OFFSET), ((in_regl(mcbsp_base_addr + MCBSP_SPCR2_OFFSET) | (0x1))));

                printk("McBSP wr failed \n");
                return -1;
            }
        }
    }

    ret_status = 0;	/* success */
    return ret_status;

}



/*-----------------------------------------------------------------------------
| Function    : get_mcbsp_base_address()
+------------------------------------------------------------------------------
| Description : returns base address of McBSP 
|
| Parameters  : <instance of McBSP>
|
| Returns     : address 
+-----------------------------------------------------------------------------*/
U32 get_mcbsp_base_address(U8 sid)
{
	U32 mcbsp_base_address = 0 ; 
	switch (sid)
    {
	    case ARM_MCBSP1: /*McBSP1*/
	        /* Update the physical addr */
	        mcbsp_base_address = ARM_MCBSP1_BASE_ADDR;
	        break;

	    case ARM_MCBSP2: /* McBSP2 */
	        /* Update the physical addr */
	        mcbsp_base_address = ARM_MCBSP2_BASE_ADDR;
	        break;

		case ARM_MCBSP3: /* McBSP3 */
	        /* Update the physical addr */
	        mcbsp_base_address = ARM_MCBSP3_BASE_ADDR;
	        break;
		case ARM_MCBSP4: /* McBSP4 */
	        /* Update the physical addr */
	        mcbsp_base_address = ARM_MCBSP4_BASE_ADDR;
	        break;
	    default:
	        mcbsp_base_address = 0;
			break;
    }
	return mcbsp_base_address;
}
static struct timer_list rx_timer;
static u32 base_addr = 0;
static void rx_handler( unsigned long data )
{
	int ret=0;
	u16 val = 0x00;
	ret = read_data_mcbsp (base_addr, &val);	

	if ((ret == 0) && (read_cnt != 128))
	{
		mod_timer( &rx_timer, jiffies + usecs_to_jiffies(100) );
		if (read_cnt == 1)
			printk ("Read data:");
		if (read_cnt != 128)
			printk (" %d", val);
	}
	else if (read_cnt == 128)
	{
		read_cnt = 0;
		printk ("\n");
		return;
	}
}

void timer_setup (void)
{
	init_timer( &rx_timer );
	setup_timer( &rx_timer, rx_handler, 0);

	mod_timer( &rx_timer, jiffies + usecs_to_jiffies(100) );
}

int test_mcbsp (int sid)
{
	u32 mcbsp_base_addr = 0;
	u16 data = 0x55AA;
	int i=0;
	read_cnt = 0;
	if ((sid < 3) || (sid > 4))
		return -1;
	mcbsp_base_addr = get_mcbsp_base_address (sid);
	base_addr = mcbsp_base_addr;
	mdelay (10);
	enable_mcbsp_clks (sid);
	reset_mcbsp (mcbsp_base_addr, 0);
	//printk ("Configuring mcbsp\n");
	config_mcbsp(mcbsp_base_addr,
                1,
                48000,						/* sampling rate value/ bit clk value	*/
                0,
				sid
               );
	//printk ("Done\n");

	//timer_setup ();
	printk ("Writing data\n");

	out_regl((mcbsp_base_addr + MCBSP_SPCR2_OFFSET),
		(in_regl((mcbsp_base_addr + MCBSP_SPCR2_OFFSET)) | (1 << 7)));
	out_regl((mcbsp_base_addr + MCBSP_SPCR2_OFFSET),
		(in_regl((mcbsp_base_addr + MCBSP_SPCR2_OFFSET)) | 0x1));
	mdelay (10);
	for (i=0; i<128; i++)
	{
		data = i+1;
		write_data_mcbsp(mcbsp_base_addr,&data);
	}
	timer_setup ();
	printk ("Done\n");
	return 0;
}
EXPORT_SYMBOL (test_mcbsp);

static int __init test_mcbsp_init(void)
{
	//test_mcbsp ();
	return 0;
}
module_init(test_mcbsp_init);

static void __exit test_mcbsp_exit(void)
{
	return;
}
module_exit(test_mcbsp_exit);

MODULE_AUTHOR("Satish Kanade <satish@mistralsolutions.com>");
MODULE_LICENSE("GPL");

/* End of Functions */


