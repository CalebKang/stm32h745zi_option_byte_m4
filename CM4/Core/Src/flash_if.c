#include "main.h"
#include "stdio.h"
#include "string.h"

uint32_t Reboot_Core=OB_REBOOT_ONLY_BOTH;

#define FLASH_OPTSR_PRG_BOOT_CM4	(FLASH_OPTSR_BCM4)
#define FLASH_OPTSR_PRG_BOOT_CM7	(FLASH_OPTSR_BCM7)

int _FLASH_ProgramTest(uint32_t u32Addr, uint32_t *p_pu32Data, uint32_t p32Length)
{
  uint8_t i = 0;
  uint8_t i_max = ((((p32Length-1)/32)+1)*32);
  uint8_t j = 0;
  uint32_t pu32Data[64];

  int32_t status = 0;
  uint8_t u08index = FLASH_NB_32BITWORD_IN_FLASHWORD;
  uint8_t u08addroffset = 0;

  if(p32Length > 256)
	{
		Error_Handler();
	}

  memset(&pu32Data, 0xff, sizeof(pu32Data)); 

  for(i=0; i<i_max; i+=4)
  {
    if(i < p32Length)
    {
      pu32Data[j] = p_pu32Data[j];
    }
    else
    {
      pu32Data[j] = 0xFFFFFFFF;
    }
    //------------------
    j++;
  }

  j = 0;

  _FLASH_Unlock();              // Flash E/W UnLock
	
	//erase 
	_FLASH_WaitForLastOperation(0xffff, FLASH_BANK_2);
	FLASH->CR2 &= ~(FLASH_CR_PSIZE | FLASH_CR_SNB);
	FLASH->CR2 |= (FLASH_CR_SER | FLASH_CR_PSIZE | (GetSector(u32Addr) << FLASH_CR_SNB_Pos) | FLASH_CR_START);
	status = _FLASH_WaitForLastOperation(0xFFFFFFFF, FLASH_BANK_2);
	FLASH->CR2 &= (~(FLASH_CR_SER | FLASH_CR_SNB));
	
	if( status != 0 )
	{
		Error_Handler();
	}

  for(i=0; i<i_max; i+=32)
  {
		//status = FLASH_BANK2_WaitForLastOperation( 0xffff );
		status = _FLASH_WaitForLastOperation(0xffff, FLASH_BANK_2);
		if( status != 0 )
		{
			Error_Handler();
		}

		SET_BIT( FLASH->CR2, FLASH_CR_PG );

		__ISB( );
		__DSB( );

		for( u08index = 0; u08index < FLASH_NB_32BITWORD_IN_FLASHWORD ; u08index++ )
		{
			*(uint32_t*)(u32Addr + u08addroffset)= pu32Data[j];
			u08addroffset+=4;
			j++;
		}
		__ISB( );
		__DSB( );

		//status = FLASH_BANK2_WaitForLastOperation(0xffff);
		status = _FLASH_WaitForLastOperation(0xffff, FLASH_BANK_2);

		CLEAR_BIT( FLASH->CR2, FLASH_CR_PG );
  }
  _FLASH_Lock();
	
	return 0;
}


int _FLASH_OptionByteTest(void)
{
  int8_t status = 0;
  int32_t reg_temp = 0;

  __disable_irq();

  _FLASH_OB_Unlock();
  _FLASH_Unlock();

  //status = FLASH_BANK1_WaitForLastOperation( 0xffffffff );
	status = _FLASH_WaitForLastOperation(0xffffffff, FLASH_BANK_1);

  if( status != 0 )
  {
    Error_Handler();
  }
	
  //status = FLASH_BANK2_WaitForLastOperation( 0xffffffff );
	status = _FLASH_WaitForLastOperation(0xffffffff, FLASH_BANK_2);
  if( status != 0 )
  {
    Error_Handler();
  }

  reg_temp = READ_REG(FLASH->OPTSR_CUR);

  if(Reboot_Core == OB_REBOOT_ONLY_M4)
  {
    reg_temp |= FLASH_OPTSR_PRG_BOOT_CM4;
    reg_temp &= ~FLASH_OPTSR_PRG_BOOT_CM7;
  }
  else if(Reboot_Core == OB_REBOOT_ONLY_M7)
  {
    reg_temp &= ~FLASH_OPTSR_PRG_BOOT_CM4;
    reg_temp |= FLASH_OPTSR_PRG_BOOT_CM7;
  }
  else
  {
    reg_temp |= ( FLASH_OPTSR_PRG_BOOT_CM7 | FLASH_OPTSR_PRG_BOOT_CM4 );
  }

  if( _FLASH_CRC_WaitForLastOperation((uint32_t)50000, 1) != 0 )
  {
    Error_Handler();
  }
  else if( _FLASH_CRC_WaitForLastOperation((uint32_t)50000, 2) != 0 )
  {
    Error_Handler();
  }
  else
  {
		//* 4) Reboot Core bit ??
		MODIFY_REG(FLASH->OPTSR_PRG, (FLASH_OPTSR_PRG_BOOT_CM7 | FLASH_OPTSR_PRG_BOOT_CM4) , reg_temp);

		//status = FLASH_BANK1_WaitForLastOperation( 0xffffffff );
	  status = _FLASH_WaitForLastOperation(0xffffffff, FLASH_BANK_1);	
		if( status != 0 )
		{
			Error_Handler();
		}
		
		//status = FLASH_BANK2_WaitForLastOperation( 0xffffffff );
	  status = _FLASH_WaitForLastOperation(0xffffffff, FLASH_BANK_2);	
		if( status != 0 )
		{
			Error_Handler();
		}
		/* Set OPTSTRT Bit */
		SET_BIT(FLASH->OPTCR, FLASH_OPTCR_OPTSTART);
		status = _FLASH_OB_WaitForLastOperation( 0xffffffff );
		if( status != 0 )
		{
			Error_Handler();
    }
  }

  _FLASH_OB_Lock();
  _FLASH_Lock();
  __enable_irq();
	
	
	return 0;
}








