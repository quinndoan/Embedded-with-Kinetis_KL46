typedef struct {
   uint32_t PDOR;                              /**< Port Data Output Register, offset: 0x0 */
   uint32_t PSOR;                              /**< Port Set Output Register, offset: 0x4 */
   uint32_t PCOR;                              /**< Port Clear Output Register, offset: 0x8 */
    uint32_t PTOR;                              /**< Port Toggle Output Register, offset: 0xC */
    uint32_t PDIR;                              /**< Port Data Input Register, offset: 0x10 */
    uint32_t PDDR;                              /**< Port Data Direction Register, offset: 0x14 */
} GPIO_Type;


/** FGPIO - Register Layout Typedef */
typedef struct {
  uint32_t PDOR;                              /**< Port Data Output Register, offset: 0x0 */
  uint32_t PSOR;                              /**< Port Set Output Register, offset: 0x4 */
  uint32_t PCOR;                              /**< Port Clear Output Register, offset: 0x8 */
  uint32_t PTOR;                              /**< Port Toggle Output Register, offset: 0xC */
  uint32_t PDIR;                              /**< Port Data Input Register, offset: 0x10 */
  uint32_t PDDR;                              /**< Port Data Direction Register, offset: 0x14 */
} FGPIO_Type;

/** PORT - Register Layout Typedef */
typedef struct {
   uint32_t PCR[32];                           /**< Pin Control Register n, array offset: 0x0, array step: 0x4 */
   uint32_t GPCLR;                             /**< Global Pin Control Low Register, offset: 0x80 */
   uint32_t GPCHR;                             /**< Global Pin Control High Register, offset: 0x84 */
   uint8_t RESERVED_0[24];
   uint32_t ISFR;                              /**< Interrupt Status Flag Register, offset: 0xA0 */
} PORT_Type;
