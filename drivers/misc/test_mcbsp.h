/*==== DECLARATION CONTROL =================================================*/
#ifndef MCBSP_H
#define MCBSP_H

/*==== DEFINES ============================================================*/
#define CORE_CM                     0x48004A00
#define PER_CM                      0x48005000
#define CM_AUTOIDLE1_CORE           (CORE_CM + 0x30)
#define CM_AUTOIDLE_PER             (PER_CM + 0x30)
#define CONTROL_DEVCONF0                 (0x48002274)
#define CONTROL_DEVCONF1                 (0x480022D8)
/*3430 MPU CM registers*/
#define CM_CLKEN_PLL_MPU			(MPU_CM + 0x04)
#define CM_IDLEST_PLL_MPU			(MPU_CM + 0x24)
#define CM_AUTOIDLE_PLL_MPU			(MPU_CM + 0x34)
#define CM_CLKSEL1_PLL_MPU			(MPU_CM + 0x40)
#define CM_CLKSEL2_PLL_MPU			(MPU_CM + 0x44)
#define CM_CLKSTCTRL_MPU			(MPU_CM + 0x48)
#define CM_CLKSTST_MPU				(MPU_CM + 0x4C)

/*3430 CORE CM registers*/
#define CM_FCLKEN1_CORE				(CORE_CM + 0x00)
#define CM_FCLKEN3_CORE				(CORE_CM + 0x08)
#define CM_ICLKEN1_CORE				(CORE_CM + 0x10)
#define CM_ICLKEN2_CORE				(CORE_CM + 0x14)
#define CM_ICLKEN3_CORE				(CORE_CM + 0x18)
#define CM_IDLEST1_CORE				(CORE_CM + 0x20)
#define CM_IDLEST2_CORE				(CORE_CM + 0x24)
#define CM_AUTOIDLE1_CORE			(CORE_CM + 0x30)
#define CM_AUTOIDLE2_CORE			(CORE_CM + 0x34)
#define CM_CLKSEL_CORE				(CORE_CM + 0x40)
#define CM_CLKSTCTRL_CORE			(CORE_CM + 0x48)
#define CM_CLKSTST_CORE				(CORE_CM + 0x4C)
/*3430 PER CM Registers*/
#define CM_FCLKEN_PER				(PER_CM)
#define CM_ICLKEN_PER				(PER_CM + 0x10)
#define CM_IDLEST_PER				(PER_CM + 0x20)
#define CM_AUTOIDLE_PER				(PER_CM + 0x30)
#define CM_CLKSEL_PER				(PER_CM + 0x40)
#define CM_SLEEPDEP_PER				(PER_CM + 0x44)
#define CM_CLKSTCTRL_PER			(PER_CM + 0x48)
#define CM_CLKSTST_PER				(PER_CM + 0x4C)

/*3430 WakeUp CM Registers*/
#define CM_FCLKEN_WKUP				(WKUP_CM + 0x00)
#define CM_ICLKEN_WKUP				(WKUP_CM + 0x10)
#define CM_IDLEST_WKUP				(WKUP_CM + 0x20)
#define CM_AUTOIDLE_WKUP			(WKUP_CM + 0x30)
#define CM_CLKSEL_WKUP				(WKUP_CM + 0x40)

#define ARM_MCBSP1_BASE_ADDR							(0x48074000)
#define ARM_MCBSP2_BASE_ADDR							(0x49022000)
#define ARM_MCBSP3_BASE_ADDR							(0x49024000)
#define ARM_MCBSP4_BASE_ADDR							(0x49026000)
/* Instance */ 
#define ARM_MCBSP1 		1
#define ARM_MCBSP2 		2
#define ARM_MCBSP3 		3
#define ARM_MCBSP4 		4
#define ARM_MCBSP5 		5

/* McBSP Register Offsets */
#define MCBSP_DRR_OFFSET		  0x00 
#define MCBSP_DXR_OFFSET          0x08
#define MCBSP_SPCR2_OFFSET        0x10
#define MCBSP_SPCR1_OFFSET        0x14
#define MCBSP_RCR2_OFFSET         0x18
#define MCBSP_RCR1_OFFSET         0x1C
#define MCBSP_XCR2_OFFSET         0x20
#define MCBSP_XCR1_OFFSET         0x24
#define MCBSP_SRGR2_OFFSET        0x28
#define MCBSP_SRGR1_OFFSET        0x2C
#define MCBSP_MCR2_OFFSET         0x30
#define MCBSP_MCR1_OFFSET         0x34
#define MCBSP_RCERA_OFFSET        0x38
#define MCBSP_RCERB_OFFSET        0x3C
#define MCBSP_XCERA_OFFSET        0x40
#define MCBSP_XCERB_OFFSET        0x44
#define MCBSP_PCR0_OFFSET         0x48

#define MCBSP_REV_OFFSET	 		0x7C    
#define MCBSP_RINTCLR_OFFSET		0x80
#define MCBSP_XINTCLR_OFFSET	    0x84
#define MCBSP_ROVFLCLR_OFFSET	    0x88
#define MCBSP_SYSCONFIG_OFFSET	    0x8C
#define MCBSP_THRSH2_OFFSET	        0x90
#define MCBSP_THRSH1_OFFSET	        0x94
#define MCBSP_IRQSTATUS_OFFSET	    0xA0

#define MCBSP_IRQENABLE_OFFSET	    0xA4
#define MCBSP_WAKEUPEN_OFFSET	    0xA8
#define MCBSP_XCCR_OFFSET	        0xAC
#define MCBSP_RCCR_OFFSET	        0xB0
#define MCBSP_XBUFFSTAT_OFFSET	    0xB4
#define MCBSP_RBUFFSTAT_OFFSET	    0xB8

#define BITSPERSAMPLE   16
#endif /* MCBSP_H */

