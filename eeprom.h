#define     __I     volatile const   /*!< defines 'read only' permissions     */
#define     __O     volatile         /*!< defines 'write only' permissions    */
#define     __IO    volatile         /*!< defines 'read / write' permissions  */

/*!< Signed integer types  */
typedef   signed char     int8_t;
typedef   signed short    int16_t;
typedef   signed long     int32_t;

/*!< Unsigned integer types  */
typedef unsigned char     uint8_t;
typedef unsigned short    uint16_t;
typedef unsigned long     uint32_t;

/*!< STM8 Standard Peripheral Library old types (maintained for legacy purpose) */

typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;
typedef enum {
    FLASH_MEMTYPE_PROG      = (uint8_t)0xFD, /*!< Program memory */
    FLASH_MEMTYPE_DATA      = (uint8_t)0xF7  /*!< Data EEPROM memory */
} FLASH_MemType_TypeDef;

typedef enum {
    FLASH_PROGRAMMODE_STANDARD = (uint8_t)0x00, /*!< Standard programming mode */
    FLASH_PROGRAMMODE_FAST     = (uint8_t)0x10  /*!< Fast programming mode */
} FLASH_ProgramMode_TypeDef;
typedef enum {
    FLASH_PROGRAMTIME_STANDARD = (uint8_t)0x00, /*!< Standard programming time fixed at 1/2 tprog */
    FLASH_PROGRAMTIME_TPROG    = (uint8_t)0x01  /*!< Programming time fixed at tprog */
} FLASH_ProgramTime_TypeDef;

#define FLASH_CR1_FIX         ((uint8_t)0x01) /*!< Fix programming time mask */
typedef struct FLASH_struct
{
  __IO uint8_t CR1;       /*!< Flash control register 1 */
  __IO uint8_t CR2;       /*!< Flash control register 2 */
  __IO uint8_t NCR2;      /*!< Flash complementary control register 2 */
  __IO uint8_t FPR;       /*!< Flash protection register */
  __IO uint8_t NFPR;      /*!< Flash complementary protection register */
  __IO uint8_t IAPSR;     /*!< Flash in-application programming status register */
  uint8_t RESERVED1;      /*!< Reserved byte */
  uint8_t RESERVED2;      /*!< Reserved byte */
  __IO uint8_t PUKR;      /*!< Flash program memory unprotection register */
  uint8_t RESERVED3;      /*!< Reserved byte */
  __IO uint8_t DUKR;      /*!< Data EEPROM unprotection register */
}
FLASH_TypeDef;
#define FLASH_BaseAddress       0x505A

#define FLASH ((FLASH_TypeDef *) FLASH_BaseAddress)
#define FLASH_RASS_KEY1 ((uint8_t)0x56) /*!< First RASS key */
#define FLASH_RASS_KEY2 ((uint8_t)0xAE) /*!< Second RASS key */
 #define NEAR __near
 #define PointerAttr NEAR
 #define MemoryAddressCast uint16_t
void FLASH_Unlock(FLASH_MemType_TypeDef FLASH_MemType)
{
  /* Check parameter */
  //assert_param(IS_MEMORY_TYPE_OK(FLASH_MemType));
  
  /* Unlock program memory */
  if(FLASH_MemType == FLASH_MEMTYPE_PROG)
  {
    FLASH->PUKR = FLASH_RASS_KEY1;
    FLASH->PUKR = FLASH_RASS_KEY2;
  }
  /* Unlock data memory */
  else
  {
    FLASH->DUKR = FLASH_RASS_KEY2; /* Warning: keys are reversed on data memory !!! */
    FLASH->DUKR = FLASH_RASS_KEY1;
  }
}

void FLASH_SetProgrammingTime(FLASH_ProgramTime_TypeDef FLASH_ProgTime)
{
  /* Check parameter */
 // assert_param(IS_FLASH_PROGRAM_TIME_OK(FLASH_ProgTime));
  
  FLASH->CR1 &= (uint8_t)(~FLASH_CR1_FIX);
  FLASH->CR1 |= (uint8_t)FLASH_ProgTime;
}
void FLASH_ProgramByte(uint32_t Address, uint8_t Data)
{
  /* Check parameters */
  //assert_param(IS_FLASH_ADDRESS_OK(Address));
  *(PointerAttr uint8_t*) (MemoryAddressCast)Address = Data;
}
uint8_t FLASH_ReadByte(uint32_t Address)
{
  /* Check parameter */
  //assert_param(IS_FLASH_ADDRESS_OK(Address));
  
  /* Read byte */
  return(*(PointerAttr uint8_t *) (MemoryAddressCast)Address); 
}