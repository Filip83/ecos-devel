/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   usb_hdr.h
 * Author: filip
 *
 * Created on 11. dubna 2017, 12:33
 */

#ifndef USB_HDR_H
#define USB_HDR_H
#if 1
#define USB_DEVICE_MAX_EP                           16
#define CYGNUM_DEVS_USB_KINETIS_CONFIG_ENDPOINTS    4
#define USB_SETUP_PACKET_SIZE                       256

/* big/little endian */
#define SWAP2BYTE_CONST(n) ((((n)&0x00FFU) << 8U) | (((n)&0xFF00U) >> 8U))
#define SWAP4BYTE_CONST(n) \
    ((((n)&0x000000FFU) << 24U) | (((n)&0x0000FF00U) << 8U) | (((n)&0x00FF0000U) >> 8U) | (((n)&0xFF000000U) >> 24U))

#if 0
#define USB_SHORT_TO_LITTLE_ENDIAN(n) SWAP2BYTE_CONST(n)
#define USB_LONG_TO_LITTLE_ENDIAN(n) SWAP4BYTE_CONST(n)
#define USB_SHORT_FROM_LITTLE_ENDIAN(n) SWAP2BYTE_CONST(n)
#define USB_LONG_FROM_LITTLE_ENDIAN(n) SWAP2BYTE_CONST(n)
#else
#define USB_SHORT_TO_LITTLE_ENDIAN(n) n
#define USB_LONG_TO_LITTLE_ENDIAN(n) n
#define USB_SHORT_FROM_LITTLE_ENDIAN(n) n
#define USB_LONG_FROM_LITTLE_ENDIAN(n) n
#endif 

#if !defined(MIN)
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#if !defined(MAX)
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif
/* ----------------------------------------------------------------------------
   -- USB Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup USB_Peripheral_Access_Layer USB Peripheral Access Layer
 * @{
 */

/** USB - Register Layout Typedef */
typedef struct {
  cyg_uint8 PERID;                              /**< Peripheral ID register, offset: 0x0 */
       cyg_uint8 RESERVED_0[3];
  cyg_uint8 IDCOMP;                             /**< Peripheral ID Complement register, offset: 0x4 */
       cyg_uint8 RESERVED_1[3];
  cyg_uint8 REV;                                /**< Peripheral Revision register, offset: 0x8 */
       cyg_uint8 RESERVED_2[3];
  cyg_uint8 ADDINFO;                            /**< Peripheral Additional Info register, offset: 0xC */
       cyg_uint8 RESERVED_3[3];
  cyg_uint8 OTGISTAT;                           /**< OTG Interrupt Status register, offset: 0x10 */
       cyg_uint8 RESERVED_4[3];
  cyg_uint8 OTGICR;                             /**< OTG Interrupt Control register, offset: 0x14 */
       cyg_uint8 RESERVED_5[3];
  cyg_uint8 OTGSTAT;                            /**< OTG Status register, offset: 0x18 */
       cyg_uint8 RESERVED_6[3];
  cyg_uint8 OTGCTL;                             /**< OTG Control register, offset: 0x1C */
       cyg_uint8 RESERVED_7[99];
  cyg_uint8 ISTAT;                              /**< Interrupt Status register, offset: 0x80 */
       cyg_uint8 RESERVED_8[3];
  cyg_uint8 INTEN;                              /**< Interrupt Enable register, offset: 0x84 */
       cyg_uint8 RESERVED_9[3];
  cyg_uint8 ERRSTAT;                            /**< Error Interrupt Status register, offset: 0x88 */
       cyg_uint8 RESERVED_10[3];
  cyg_uint8 ERREN;                              /**< Error Interrupt Enable register, offset: 0x8C */
       cyg_uint8 RESERVED_11[3];
  cyg_uint8 STAT;                               /**< Status register, offset: 0x90 */
       cyg_uint8 RESERVED_12[3];
  cyg_uint8 CTL;                                /**< Control register, offset: 0x94 */
       cyg_uint8 RESERVED_13[3];
  cyg_uint8 ADDR;                               /**< Address register, offset: 0x98 */
       cyg_uint8 RESERVED_14[3];
  cyg_uint8 BDTPAGE1;                           /**< BDT Page register 1, offset: 0x9C */
       cyg_uint8 RESERVED_15[3];
  cyg_uint8 FRMNUML;                            /**< Frame Number register Low, offset: 0xA0 */
       cyg_uint8 RESERVED_16[3];
  cyg_uint8 FRMNUMH;                            /**< Frame Number register High, offset: 0xA4 */
       cyg_uint8 RESERVED_17[3];
  cyg_uint8 TOKEN;                              /**< Token register, offset: 0xA8 */
       cyg_uint8 RESERVED_18[3];
  cyg_uint8 SOFTHLD;                            /**< SOF Threshold register, offset: 0xAC */
       cyg_uint8 RESERVED_19[3];
  cyg_uint8 BDTPAGE2;                           /**< BDT Page Register 2, offset: 0xB0 */
       cyg_uint8 RESERVED_20[3];
  cyg_uint8 BDTPAGE3;                           /**< BDT Page Register 3, offset: 0xB4 */
       cyg_uint8 RESERVED_21[11];
  struct {                                         /* offset: 0xC0, array step: 0x4 */
    cyg_uint8 ENDPT;                              /**< Endpoint Control register, array offset: 0xC0, array step: 0x4 */
         cyg_uint8 RESERVED_0[3];
  } ENDPOINT[16];
  cyg_uint8 USBCTRL;                            /**< USB Control register, offset: 0x100 */
       cyg_uint8 RESERVED_22[3];
  cyg_uint8 OBSERVE;                            /**< USB OTG Observe register, offset: 0x104 */
       cyg_uint8 RESERVED_23[3];
  cyg_uint8 CONTROL;                            /**< USB OTG Control register, offset: 0x108 */
       cyg_uint8 RESERVED_24[3];
  cyg_uint8 USBTRC0;                            /**< USB Transceiver Control register 0, offset: 0x10C */
       cyg_uint8 RESERVED_25[7];
  cyg_uint8 USBFRMADJUST;                       /**< Frame Adjust Register, offset: 0x114 */
       cyg_uint8 RESERVED_26[43];
  cyg_uint8 CLK_RECOVER_CTRL;                   /**< USB Clock recovery control, offset: 0x140 */
       cyg_uint8 RESERVED_27[3];
  cyg_uint8 CLK_RECOVER_IRC_EN;                 /**< IRC48M oscillator enable register, offset: 0x144 */
       cyg_uint8 RESERVED_28[23];
  cyg_uint8 CLK_RECOVER_INT_STATUS;             /**< Clock recovery separated interrupt status, offset: 0x15C */
} USB_Type;

/* ----------------------------------------------------------------------------
   -- USB Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup USB_Register_Masks USB Register Masks
 * @{
 */

/*! @name PERID - Peripheral ID register */
#define USB_PERID_ID_MASK                        (0x3FU)
#define USB_PERID_ID_SHIFT                       (0U)
#define USB_PERID_ID(x)                          (((cyg_uint8)(((cyg_uint8)(x)) << USB_PERID_ID_SHIFT)) & USB_PERID_ID_MASK)

/*! @name IDCOMP - Peripheral ID Complement register */
#define USB_IDCOMP_NID_MASK                      (0x3FU)
#define USB_IDCOMP_NID_SHIFT                     (0U)
#define USB_IDCOMP_NID(x)                        (((cyg_uint8)(((cyg_uint8)(x)) << USB_IDCOMP_NID_SHIFT)) & USB_IDCOMP_NID_MASK)

/*! @name REV - Peripheral Revision register */
#define USB_REV_REV_MASK                         (0xFFU)
#define USB_REV_REV_SHIFT                        (0U)
#define USB_REV_REV(x)                           (((cyg_uint8)(((cyg_uint8)(x)) << USB_REV_REV_SHIFT)) & USB_REV_REV_MASK)

/*! @name ADDINFO - Peripheral Additional Info register */
#define USB_ADDINFO_IEHOST_MASK                  (0x1U)
#define USB_ADDINFO_IEHOST_SHIFT                 (0U)
#define USB_ADDINFO_IEHOST(x)                    (((cyg_uint8)(((cyg_uint8)(x)) << USB_ADDINFO_IEHOST_SHIFT)) & USB_ADDINFO_IEHOST_MASK)

/*! @name OTGISTAT - OTG Interrupt Status register */
#define USB_OTGISTAT_AVBUSCHG_MASK               (0x1U)
#define USB_OTGISTAT_AVBUSCHG_SHIFT              (0U)
#define USB_OTGISTAT_AVBUSCHG(x)                 (((cyg_uint8)(((cyg_uint8)(x)) << USB_OTGISTAT_AVBUSCHG_SHIFT)) & USB_OTGISTAT_AVBUSCHG_MASK)
#define USB_OTGISTAT_B_SESS_CHG_MASK             (0x4U)
#define USB_OTGISTAT_B_SESS_CHG_SHIFT            (2U)
#define USB_OTGISTAT_B_SESS_CHG(x)               (((cyg_uint8)(((cyg_uint8)(x)) << USB_OTGISTAT_B_SESS_CHG_SHIFT)) & USB_OTGISTAT_B_SESS_CHG_MASK)
#define USB_OTGISTAT_SESSVLDCHG_MASK             (0x8U)
#define USB_OTGISTAT_SESSVLDCHG_SHIFT            (3U)
#define USB_OTGISTAT_SESSVLDCHG(x)               (((cyg_uint8)(((cyg_uint8)(x)) << USB_OTGISTAT_SESSVLDCHG_SHIFT)) & USB_OTGISTAT_SESSVLDCHG_MASK)
#define USB_OTGISTAT_LINE_STATE_CHG_MASK         (0x20U)
#define USB_OTGISTAT_LINE_STATE_CHG_SHIFT        (5U)
#define USB_OTGISTAT_LINE_STATE_CHG(x)           (((cyg_uint8)(((cyg_uint8)(x)) << USB_OTGISTAT_LINE_STATE_CHG_SHIFT)) & USB_OTGISTAT_LINE_STATE_CHG_MASK)
#define USB_OTGISTAT_ONEMSEC_MASK                (0x40U)
#define USB_OTGISTAT_ONEMSEC_SHIFT               (6U)
#define USB_OTGISTAT_ONEMSEC(x)                  (((cyg_uint8)(((cyg_uint8)(x)) << USB_OTGISTAT_ONEMSEC_SHIFT)) & USB_OTGISTAT_ONEMSEC_MASK)
#define USB_OTGISTAT_IDCHG_MASK                  (0x80U)
#define USB_OTGISTAT_IDCHG_SHIFT                 (7U)
#define USB_OTGISTAT_IDCHG(x)                    (((cyg_uint8)(((cyg_uint8)(x)) << USB_OTGISTAT_IDCHG_SHIFT)) & USB_OTGISTAT_IDCHG_MASK)

/*! @name OTGICR - OTG Interrupt Control register */
#define USB_OTGICR_AVBUSEN_MASK                  (0x1U)
#define USB_OTGICR_AVBUSEN_SHIFT                 (0U)
#define USB_OTGICR_AVBUSEN(x)                    (((cyg_uint8)(((cyg_uint8)(x)) << USB_OTGICR_AVBUSEN_SHIFT)) & USB_OTGICR_AVBUSEN_MASK)
#define USB_OTGICR_BSESSEN_MASK                  (0x4U)
#define USB_OTGICR_BSESSEN_SHIFT                 (2U)
#define USB_OTGICR_BSESSEN(x)                    (((cyg_uint8)(((cyg_uint8)(x)) << USB_OTGICR_BSESSEN_SHIFT)) & USB_OTGICR_BSESSEN_MASK)
#define USB_OTGICR_SESSVLDEN_MASK                (0x8U)
#define USB_OTGICR_SESSVLDEN_SHIFT               (3U)
#define USB_OTGICR_SESSVLDEN(x)                  (((cyg_uint8)(((cyg_uint8)(x)) << USB_OTGICR_SESSVLDEN_SHIFT)) & USB_OTGICR_SESSVLDEN_MASK)
#define USB_OTGICR_LINESTATEEN_MASK              (0x20U)
#define USB_OTGICR_LINESTATEEN_SHIFT             (5U)
#define USB_OTGICR_LINESTATEEN(x)                (((cyg_uint8)(((cyg_uint8)(x)) << USB_OTGICR_LINESTATEEN_SHIFT)) & USB_OTGICR_LINESTATEEN_MASK)
#define USB_OTGICR_ONEMSECEN_MASK                (0x40U)
#define USB_OTGICR_ONEMSECEN_SHIFT               (6U)
#define USB_OTGICR_ONEMSECEN(x)                  (((cyg_uint8)(((cyg_uint8)(x)) << USB_OTGICR_ONEMSECEN_SHIFT)) & USB_OTGICR_ONEMSECEN_MASK)
#define USB_OTGICR_IDEN_MASK                     (0x80U)
#define USB_OTGICR_IDEN_SHIFT                    (7U)
#define USB_OTGICR_IDEN(x)                       (((cyg_uint8)(((cyg_uint8)(x)) << USB_OTGICR_IDEN_SHIFT)) & USB_OTGICR_IDEN_MASK)

/*! @name OTGSTAT - OTG Status register */
#define USB_OTGSTAT_AVBUSVLD_MASK                (0x1U)
#define USB_OTGSTAT_AVBUSVLD_SHIFT               (0U)
#define USB_OTGSTAT_AVBUSVLD(x)                  (((cyg_uint8)(((cyg_uint8)(x)) << USB_OTGSTAT_AVBUSVLD_SHIFT)) & USB_OTGSTAT_AVBUSVLD_MASK)
#define USB_OTGSTAT_BSESSEND_MASK                (0x4U)
#define USB_OTGSTAT_BSESSEND_SHIFT               (2U)
#define USB_OTGSTAT_BSESSEND(x)                  (((cyg_uint8)(((cyg_uint8)(x)) << USB_OTGSTAT_BSESSEND_SHIFT)) & USB_OTGSTAT_BSESSEND_MASK)
#define USB_OTGSTAT_SESS_VLD_MASK                (0x8U)
#define USB_OTGSTAT_SESS_VLD_SHIFT               (3U)
#define USB_OTGSTAT_SESS_VLD(x)                  (((cyg_uint8)(((cyg_uint8)(x)) << USB_OTGSTAT_SESS_VLD_SHIFT)) & USB_OTGSTAT_SESS_VLD_MASK)
#define USB_OTGSTAT_LINESTATESTABLE_MASK         (0x20U)
#define USB_OTGSTAT_LINESTATESTABLE_SHIFT        (5U)
#define USB_OTGSTAT_LINESTATESTABLE(x)           (((cyg_uint8)(((cyg_uint8)(x)) << USB_OTGSTAT_LINESTATESTABLE_SHIFT)) & USB_OTGSTAT_LINESTATESTABLE_MASK)
#define USB_OTGSTAT_ONEMSECEN_MASK               (0x40U)
#define USB_OTGSTAT_ONEMSECEN_SHIFT              (6U)
#define USB_OTGSTAT_ONEMSECEN(x)                 (((cyg_uint8)(((cyg_uint8)(x)) << USB_OTGSTAT_ONEMSECEN_SHIFT)) & USB_OTGSTAT_ONEMSECEN_MASK)
#define USB_OTGSTAT_ID_MASK                      (0x80U)
#define USB_OTGSTAT_ID_SHIFT                     (7U)
#define USB_OTGSTAT_ID(x)                        (((cyg_uint8)(((cyg_uint8)(x)) << USB_OTGSTAT_ID_SHIFT)) & USB_OTGSTAT_ID_MASK)

/*! @name OTGCTL - OTG Control register */
#define USB_OTGCTL_OTGEN_MASK                    (0x4U)
#define USB_OTGCTL_OTGEN_SHIFT                   (2U)
#define USB_OTGCTL_OTGEN(x)                      (((cyg_uint8)(((cyg_uint8)(x)) << USB_OTGCTL_OTGEN_SHIFT)) & USB_OTGCTL_OTGEN_MASK)
#define USB_OTGCTL_DMLOW_MASK                    (0x10U)
#define USB_OTGCTL_DMLOW_SHIFT                   (4U)
#define USB_OTGCTL_DMLOW(x)                      (((cyg_uint8)(((cyg_uint8)(x)) << USB_OTGCTL_DMLOW_SHIFT)) & USB_OTGCTL_DMLOW_MASK)
#define USB_OTGCTL_DPLOW_MASK                    (0x20U)
#define USB_OTGCTL_DPLOW_SHIFT                   (5U)
#define USB_OTGCTL_DPLOW(x)                      (((cyg_uint8)(((cyg_uint8)(x)) << USB_OTGCTL_DPLOW_SHIFT)) & USB_OTGCTL_DPLOW_MASK)
#define USB_OTGCTL_DPHIGH_MASK                   (0x80U)
#define USB_OTGCTL_DPHIGH_SHIFT                  (7U)
#define USB_OTGCTL_DPHIGH(x)                     (((cyg_uint8)(((cyg_uint8)(x)) << USB_OTGCTL_DPHIGH_SHIFT)) & USB_OTGCTL_DPHIGH_MASK)

/*! @name ISTAT - Interrupt Status register */
#define USB_ISTAT_USBRST_MASK                    (0x1U)
#define USB_ISTAT_USBRST_SHIFT                   (0U)
#define USB_ISTAT_USBRST(x)                      (((cyg_uint8)(((cyg_uint8)(x)) << USB_ISTAT_USBRST_SHIFT)) & USB_ISTAT_USBRST_MASK)
#define USB_ISTAT_ERROR_MASK                     (0x2U)
#define USB_ISTAT_ERROR_SHIFT                    (1U)
#define USB_ISTAT_ERROR(x)                       (((cyg_uint8)(((cyg_uint8)(x)) << USB_ISTAT_ERROR_SHIFT)) & USB_ISTAT_ERROR_MASK)
#define USB_ISTAT_SOFTOK_MASK                    (0x4U)
#define USB_ISTAT_SOFTOK_SHIFT                   (2U)
#define USB_ISTAT_SOFTOK(x)                      (((cyg_uint8)(((cyg_uint8)(x)) << USB_ISTAT_SOFTOK_SHIFT)) & USB_ISTAT_SOFTOK_MASK)
#define USB_ISTAT_TOKDNE_MASK                    (0x8U)
#define USB_ISTAT_TOKDNE_SHIFT                   (3U)
#define USB_ISTAT_TOKDNE(x)                      (((cyg_uint8)(((cyg_uint8)(x)) << USB_ISTAT_TOKDNE_SHIFT)) & USB_ISTAT_TOKDNE_MASK)
#define USB_ISTAT_SLEEP_MASK                     (0x10U)
#define USB_ISTAT_SLEEP_SHIFT                    (4U)
#define USB_ISTAT_SLEEP(x)                       (((cyg_uint8)(((cyg_uint8)(x)) << USB_ISTAT_SLEEP_SHIFT)) & USB_ISTAT_SLEEP_MASK)
#define USB_ISTAT_RESUME_MASK                    (0x20U)
#define USB_ISTAT_RESUME_SHIFT                   (5U)
#define USB_ISTAT_RESUME(x)                      (((cyg_uint8)(((cyg_uint8)(x)) << USB_ISTAT_RESUME_SHIFT)) & USB_ISTAT_RESUME_MASK)
#define USB_ISTAT_ATTACH_MASK                    (0x40U)
#define USB_ISTAT_ATTACH_SHIFT                   (6U)
#define USB_ISTAT_ATTACH(x)                      (((cyg_uint8)(((cyg_uint8)(x)) << USB_ISTAT_ATTACH_SHIFT)) & USB_ISTAT_ATTACH_MASK)
#define USB_ISTAT_STALL_MASK                     (0x80U)
#define USB_ISTAT_STALL_SHIFT                    (7U)
#define USB_ISTAT_STALL(x)                       (((cyg_uint8)(((cyg_uint8)(x)) << USB_ISTAT_STALL_SHIFT)) & USB_ISTAT_STALL_MASK)

/*! @name INTEN - Interrupt Enable register */
#define USB_INTEN_USBRSTEN_MASK                  (0x1U)
#define USB_INTEN_USBRSTEN_SHIFT                 (0U)
#define USB_INTEN_USBRSTEN(x)                    (((cyg_uint8)(((cyg_uint8)(x)) << USB_INTEN_USBRSTEN_SHIFT)) & USB_INTEN_USBRSTEN_MASK)
#define USB_INTEN_ERROREN_MASK                   (0x2U)
#define USB_INTEN_ERROREN_SHIFT                  (1U)
#define USB_INTEN_ERROREN(x)                     (((cyg_uint8)(((cyg_uint8)(x)) << USB_INTEN_ERROREN_SHIFT)) & USB_INTEN_ERROREN_MASK)
#define USB_INTEN_SOFTOKEN_MASK                  (0x4U)
#define USB_INTEN_SOFTOKEN_SHIFT                 (2U)
#define USB_INTEN_SOFTOKEN(x)                    (((cyg_uint8)(((cyg_uint8)(x)) << USB_INTEN_SOFTOKEN_SHIFT)) & USB_INTEN_SOFTOKEN_MASK)
#define USB_INTEN_TOKDNEEN_MASK                  (0x8U)
#define USB_INTEN_TOKDNEEN_SHIFT                 (3U)
#define USB_INTEN_TOKDNEEN(x)                    (((cyg_uint8)(((cyg_uint8)(x)) << USB_INTEN_TOKDNEEN_SHIFT)) & USB_INTEN_TOKDNEEN_MASK)
#define USB_INTEN_SLEEPEN_MASK                   (0x10U)
#define USB_INTEN_SLEEPEN_SHIFT                  (4U)
#define USB_INTEN_SLEEPEN(x)                     (((cyg_uint8)(((cyg_uint8)(x)) << USB_INTEN_SLEEPEN_SHIFT)) & USB_INTEN_SLEEPEN_MASK)
#define USB_INTEN_RESUMEEN_MASK                  (0x20U)
#define USB_INTEN_RESUMEEN_SHIFT                 (5U)
#define USB_INTEN_RESUMEEN(x)                    (((cyg_uint8)(((cyg_uint8)(x)) << USB_INTEN_RESUMEEN_SHIFT)) & USB_INTEN_RESUMEEN_MASK)
#define USB_INTEN_ATTACHEN_MASK                  (0x40U)
#define USB_INTEN_ATTACHEN_SHIFT                 (6U)
#define USB_INTEN_ATTACHEN(x)                    (((cyg_uint8)(((cyg_uint8)(x)) << USB_INTEN_ATTACHEN_SHIFT)) & USB_INTEN_ATTACHEN_MASK)
#define USB_INTEN_STALLEN_MASK                   (0x80U)
#define USB_INTEN_STALLEN_SHIFT                  (7U)
#define USB_INTEN_STALLEN(x)                     (((cyg_uint8)(((cyg_uint8)(x)) << USB_INTEN_STALLEN_SHIFT)) & USB_INTEN_STALLEN_MASK)

/*! @name ERRSTAT - Error Interrupt Status register */
#define USB_ERRSTAT_PIDERR_MASK                  (0x1U)
#define USB_ERRSTAT_PIDERR_SHIFT                 (0U)
#define USB_ERRSTAT_PIDERR(x)                    (((cyg_uint8)(((cyg_uint8)(x)) << USB_ERRSTAT_PIDERR_SHIFT)) & USB_ERRSTAT_PIDERR_MASK)
#define USB_ERRSTAT_CRC5EOF_MASK                 (0x2U)
#define USB_ERRSTAT_CRC5EOF_SHIFT                (1U)
#define USB_ERRSTAT_CRC5EOF(x)                   (((cyg_uint8)(((cyg_uint8)(x)) << USB_ERRSTAT_CRC5EOF_SHIFT)) & USB_ERRSTAT_CRC5EOF_MASK)
#define USB_ERRSTAT_CRC16_MASK                   (0x4U)
#define USB_ERRSTAT_CRC16_SHIFT                  (2U)
#define USB_ERRSTAT_CRC16(x)                     (((cyg_uint8)(((cyg_uint8)(x)) << USB_ERRSTAT_CRC16_SHIFT)) & USB_ERRSTAT_CRC16_MASK)
#define USB_ERRSTAT_DFN8_MASK                    (0x8U)
#define USB_ERRSTAT_DFN8_SHIFT                   (3U)
#define USB_ERRSTAT_DFN8(x)                      (((cyg_uint8)(((cyg_uint8)(x)) << USB_ERRSTAT_DFN8_SHIFT)) & USB_ERRSTAT_DFN8_MASK)
#define USB_ERRSTAT_BTOERR_MASK                  (0x10U)
#define USB_ERRSTAT_BTOERR_SHIFT                 (4U)
#define USB_ERRSTAT_BTOERR(x)                    (((cyg_uint8)(((cyg_uint8)(x)) << USB_ERRSTAT_BTOERR_SHIFT)) & USB_ERRSTAT_BTOERR_MASK)
#define USB_ERRSTAT_DMAERR_MASK                  (0x20U)
#define USB_ERRSTAT_DMAERR_SHIFT                 (5U)
#define USB_ERRSTAT_DMAERR(x)                    (((cyg_uint8)(((cyg_uint8)(x)) << USB_ERRSTAT_DMAERR_SHIFT)) & USB_ERRSTAT_DMAERR_MASK)
#define USB_ERRSTAT_BTSERR_MASK                  (0x80U)
#define USB_ERRSTAT_BTSERR_SHIFT                 (7U)
#define USB_ERRSTAT_BTSERR(x)                    (((cyg_uint8)(((cyg_uint8)(x)) << USB_ERRSTAT_BTSERR_SHIFT)) & USB_ERRSTAT_BTSERR_MASK)

/*! @name ERREN - Error Interrupt Enable register */
#define USB_ERREN_PIDERREN_MASK                  (0x1U)
#define USB_ERREN_PIDERREN_SHIFT                 (0U)
#define USB_ERREN_PIDERREN(x)                    (((cyg_uint8)(((cyg_uint8)(x)) << USB_ERREN_PIDERREN_SHIFT)) & USB_ERREN_PIDERREN_MASK)
#define USB_ERREN_CRC5EOFEN_MASK                 (0x2U)
#define USB_ERREN_CRC5EOFEN_SHIFT                (1U)
#define USB_ERREN_CRC5EOFEN(x)                   (((cyg_uint8)(((cyg_uint8)(x)) << USB_ERREN_CRC5EOFEN_SHIFT)) & USB_ERREN_CRC5EOFEN_MASK)
#define USB_ERREN_CRC16EN_MASK                   (0x4U)
#define USB_ERREN_CRC16EN_SHIFT                  (2U)
#define USB_ERREN_CRC16EN(x)                     (((cyg_uint8)(((cyg_uint8)(x)) << USB_ERREN_CRC16EN_SHIFT)) & USB_ERREN_CRC16EN_MASK)
#define USB_ERREN_DFN8EN_MASK                    (0x8U)
#define USB_ERREN_DFN8EN_SHIFT                   (3U)
#define USB_ERREN_DFN8EN(x)                      (((cyg_uint8)(((cyg_uint8)(x)) << USB_ERREN_DFN8EN_SHIFT)) & USB_ERREN_DFN8EN_MASK)
#define USB_ERREN_BTOERREN_MASK                  (0x10U)
#define USB_ERREN_BTOERREN_SHIFT                 (4U)
#define USB_ERREN_BTOERREN(x)                    (((cyg_uint8)(((cyg_uint8)(x)) << USB_ERREN_BTOERREN_SHIFT)) & USB_ERREN_BTOERREN_MASK)
#define USB_ERREN_DMAERREN_MASK                  (0x20U)
#define USB_ERREN_DMAERREN_SHIFT                 (5U)
#define USB_ERREN_DMAERREN(x)                    (((cyg_uint8)(((cyg_uint8)(x)) << USB_ERREN_DMAERREN_SHIFT)) & USB_ERREN_DMAERREN_MASK)
#define USB_ERREN_BTSERREN_MASK                  (0x80U)
#define USB_ERREN_BTSERREN_SHIFT                 (7U)
#define USB_ERREN_BTSERREN(x)                    (((cyg_uint8)(((cyg_uint8)(x)) << USB_ERREN_BTSERREN_SHIFT)) & USB_ERREN_BTSERREN_MASK)

/*! @name STAT - Status register */
#define USB_STAT_ODD_MASK                        (0x4U)
#define USB_STAT_ODD_SHIFT                       (2U)
#define USB_STAT_ODD(x)                          (((cyg_uint8)(((cyg_uint8)(x)) << USB_STAT_ODD_SHIFT)) & USB_STAT_ODD_MASK)
#define USB_STAT_TX_MASK                         (0x8U)
#define USB_STAT_TX_SHIFT                        (3U)
#define USB_STAT_TX(x)                           (((cyg_uint8)(((cyg_uint8)(x)) << USB_STAT_TX_SHIFT)) & USB_STAT_TX_MASK)
#define USB_STAT_ENDP_MASK                       (0xF0U)
#define USB_STAT_ENDP_SHIFT                      (4U)
#define USB_STAT_ENDP(x)                         (((cyg_uint8)(((cyg_uint8)(x)) << USB_STAT_ENDP_SHIFT)) & USB_STAT_ENDP_MASK)

/*! @name CTL - Control register */
#define USB_CTL_USBENSOFEN_MASK                  (0x1U)
#define USB_CTL_USBENSOFEN_SHIFT                 (0U)
#define USB_CTL_USBENSOFEN(x)                    (((cyg_uint8)(((cyg_uint8)(x)) << USB_CTL_USBENSOFEN_SHIFT)) & USB_CTL_USBENSOFEN_MASK)
#define USB_CTL_ODDRST_MASK                      (0x2U)
#define USB_CTL_ODDRST_SHIFT                     (1U)
#define USB_CTL_ODDRST(x)                        (((cyg_uint8)(((cyg_uint8)(x)) << USB_CTL_ODDRST_SHIFT)) & USB_CTL_ODDRST_MASK)
#define USB_CTL_RESUME_MASK                      (0x4U)
#define USB_CTL_RESUME_SHIFT                     (2U)
#define USB_CTL_RESUME(x)                        (((cyg_uint8)(((cyg_uint8)(x)) << USB_CTL_RESUME_SHIFT)) & USB_CTL_RESUME_MASK)
#define USB_CTL_HOSTMODEEN_MASK                  (0x8U)
#define USB_CTL_HOSTMODEEN_SHIFT                 (3U)
#define USB_CTL_HOSTMODEEN(x)                    (((cyg_uint8)(((cyg_uint8)(x)) << USB_CTL_HOSTMODEEN_SHIFT)) & USB_CTL_HOSTMODEEN_MASK)
#define USB_CTL_RESET_MASK                       (0x10U)
#define USB_CTL_RESET_SHIFT                      (4U)
#define USB_CTL_RESET(x)                         (((cyg_uint8)(((cyg_uint8)(x)) << USB_CTL_RESET_SHIFT)) & USB_CTL_RESET_MASK)
#define USB_CTL_TXSUSPENDTOKENBUSY_MASK          (0x20U)
#define USB_CTL_TXSUSPENDTOKENBUSY_SHIFT         (5U)
#define USB_CTL_TXSUSPENDTOKENBUSY(x)            (((cyg_uint8)(((cyg_uint8)(x)) << USB_CTL_TXSUSPENDTOKENBUSY_SHIFT)) & USB_CTL_TXSUSPENDTOKENBUSY_MASK)
#define USB_CTL_SE0_MASK                         (0x40U)
#define USB_CTL_SE0_SHIFT                        (6U)
#define USB_CTL_SE0(x)                           (((cyg_uint8)(((cyg_uint8)(x)) << USB_CTL_SE0_SHIFT)) & USB_CTL_SE0_MASK)
#define USB_CTL_JSTATE_MASK                      (0x80U)
#define USB_CTL_JSTATE_SHIFT                     (7U)
#define USB_CTL_JSTATE(x)                        (((cyg_uint8)(((cyg_uint8)(x)) << USB_CTL_JSTATE_SHIFT)) & USB_CTL_JSTATE_MASK)

/*! @name ADDR - Address register */
#define USB_ADDR_ADDR_MASK                       (0x7FU)
#define USB_ADDR_ADDR_SHIFT                      (0U)
#define USB_ADDR_ADDR(x)                         (((cyg_uint8)(((cyg_uint8)(x)) << USB_ADDR_ADDR_SHIFT)) & USB_ADDR_ADDR_MASK)
#define USB_ADDR_LSEN_MASK                       (0x80U)
#define USB_ADDR_LSEN_SHIFT                      (7U)
#define USB_ADDR_LSEN(x)                         (((cyg_uint8)(((cyg_uint8)(x)) << USB_ADDR_LSEN_SHIFT)) & USB_ADDR_LSEN_MASK)

/*! @name BDTPAGE1 - BDT Page register 1 */
#define USB_BDTPAGE1_BDTBA_MASK                  (0xFEU)
#define USB_BDTPAGE1_BDTBA_SHIFT                 (1U)
#define USB_BDTPAGE1_BDTBA(x)                    (((cyg_uint8)(((cyg_uint8)(x)) << USB_BDTPAGE1_BDTBA_SHIFT)) & USB_BDTPAGE1_BDTBA_MASK)

/*! @name FRMNUML - Frame Number register Low */
#define USB_FRMNUML_FRM_MASK                     (0xFFU)
#define USB_FRMNUML_FRM_SHIFT                    (0U)
#define USB_FRMNUML_FRM(x)                       (((cyg_uint8)(((cyg_uint8)(x)) << USB_FRMNUML_FRM_SHIFT)) & USB_FRMNUML_FRM_MASK)

/*! @name FRMNUMH - Frame Number register High */
#define USB_FRMNUMH_FRM_MASK                     (0x7U)
#define USB_FRMNUMH_FRM_SHIFT                    (0U)
#define USB_FRMNUMH_FRM(x)                       (((cyg_uint8)(((cyg_uint8)(x)) << USB_FRMNUMH_FRM_SHIFT)) & USB_FRMNUMH_FRM_MASK)

/*! @name TOKEN - Token register */
#define USB_TOKEN_TOKENENDPT_MASK                (0xFU)
#define USB_TOKEN_TOKENENDPT_SHIFT               (0U)
#define USB_TOKEN_TOKENENDPT(x)                  (((cyg_uint8)(((cyg_uint8)(x)) << USB_TOKEN_TOKENENDPT_SHIFT)) & USB_TOKEN_TOKENENDPT_MASK)
#define USB_TOKEN_TOKENPID_MASK                  (0xF0U)
#define USB_TOKEN_TOKENPID_SHIFT                 (4U)
#define USB_TOKEN_TOKENPID(x)                    (((cyg_uint8)(((cyg_uint8)(x)) << USB_TOKEN_TOKENPID_SHIFT)) & USB_TOKEN_TOKENPID_MASK)

/*! @name SOFTHLD - SOF Threshold register */
#define USB_SOFTHLD_CNT_MASK                     (0xFFU)
#define USB_SOFTHLD_CNT_SHIFT                    (0U)
#define USB_SOFTHLD_CNT(x)                       (((cyg_uint8)(((cyg_uint8)(x)) << USB_SOFTHLD_CNT_SHIFT)) & USB_SOFTHLD_CNT_MASK)

/*! @name BDTPAGE2 - BDT Page Register 2 */
#define USB_BDTPAGE2_BDTBA_MASK                  (0xFFU)
#define USB_BDTPAGE2_BDTBA_SHIFT                 (0U)
#define USB_BDTPAGE2_BDTBA(x)                    (((cyg_uint8)(((cyg_uint8)(x)) << USB_BDTPAGE2_BDTBA_SHIFT)) & USB_BDTPAGE2_BDTBA_MASK)

/*! @name BDTPAGE3 - BDT Page Register 3 */
#define USB_BDTPAGE3_BDTBA_MASK                  (0xFFU)
#define USB_BDTPAGE3_BDTBA_SHIFT                 (0U)
#define USB_BDTPAGE3_BDTBA(x)                    (((cyg_uint8)(((cyg_uint8)(x)) << USB_BDTPAGE3_BDTBA_SHIFT)) & USB_BDTPAGE3_BDTBA_MASK)

/*! @name ENDPT - Endpoint Control register */
#define USB_ENDPT_EPHSHK_MASK                    (0x1U)
#define USB_ENDPT_EPHSHK_SHIFT                   (0U)
#define USB_ENDPT_EPHSHK(x)                      (((cyg_uint8)(((cyg_uint8)(x)) << USB_ENDPT_EPHSHK_SHIFT)) & USB_ENDPT_EPHSHK_MASK)
#define USB_ENDPT_EPSTALL_MASK                   (0x2U)
#define USB_ENDPT_EPSTALL_SHIFT                  (1U)
#define USB_ENDPT_EPSTALL(x)                     (((cyg_uint8)(((cyg_uint8)(x)) << USB_ENDPT_EPSTALL_SHIFT)) & USB_ENDPT_EPSTALL_MASK)
#define USB_ENDPT_EPTXEN_MASK                    (0x4U)
#define USB_ENDPT_EPTXEN_SHIFT                   (2U)
#define USB_ENDPT_EPTXEN(x)                      (((cyg_uint8)(((cyg_uint8)(x)) << USB_ENDPT_EPTXEN_SHIFT)) & USB_ENDPT_EPTXEN_MASK)
#define USB_ENDPT_EPRXEN_MASK                    (0x8U)
#define USB_ENDPT_EPRXEN_SHIFT                   (3U)
#define USB_ENDPT_EPRXEN(x)                      (((cyg_uint8)(((cyg_uint8)(x)) << USB_ENDPT_EPRXEN_SHIFT)) & USB_ENDPT_EPRXEN_MASK)
#define USB_ENDPT_EPCTLDIS_MASK                  (0x10U)
#define USB_ENDPT_EPCTLDIS_SHIFT                 (4U)
#define USB_ENDPT_EPCTLDIS(x)                    (((cyg_uint8)(((cyg_uint8)(x)) << USB_ENDPT_EPCTLDIS_SHIFT)) & USB_ENDPT_EPCTLDIS_MASK)
#define USB_ENDPT_RETRYDIS_MASK                  (0x40U)
#define USB_ENDPT_RETRYDIS_SHIFT                 (6U)
#define USB_ENDPT_RETRYDIS(x)                    (((cyg_uint8)(((cyg_uint8)(x)) << USB_ENDPT_RETRYDIS_SHIFT)) & USB_ENDPT_RETRYDIS_MASK)
#define USB_ENDPT_HOSTWOHUB_MASK                 (0x80U)
#define USB_ENDPT_HOSTWOHUB_SHIFT                (7U)
#define USB_ENDPT_HOSTWOHUB(x)                   (((cyg_uint8)(((cyg_uint8)(x)) << USB_ENDPT_HOSTWOHUB_SHIFT)) & USB_ENDPT_HOSTWOHUB_MASK)

/* The count of USB_ENDPT */
#define USB_ENDPT_COUNT                          (16U)

/*! @name USBCTRL - USB Control register */
#define USB_USBCTRL_PDE_MASK                     (0x40U)
#define USB_USBCTRL_PDE_SHIFT                    (6U)
#define USB_USBCTRL_PDE(x)                       (((cyg_uint8)(((cyg_uint8)(x)) << USB_USBCTRL_PDE_SHIFT)) & USB_USBCTRL_PDE_MASK)
#define USB_USBCTRL_SUSP_MASK                    (0x80U)
#define USB_USBCTRL_SUSP_SHIFT                   (7U)
#define USB_USBCTRL_SUSP(x)                      (((cyg_uint8)(((cyg_uint8)(x)) << USB_USBCTRL_SUSP_SHIFT)) & USB_USBCTRL_SUSP_MASK)

/*! @name OBSERVE - USB OTG Observe register */
#define USB_OBSERVE_DMPD_MASK                    (0x10U)
#define USB_OBSERVE_DMPD_SHIFT                   (4U)
#define USB_OBSERVE_DMPD(x)                      (((cyg_uint8)(((cyg_uint8)(x)) << USB_OBSERVE_DMPD_SHIFT)) & USB_OBSERVE_DMPD_MASK)
#define USB_OBSERVE_DPPD_MASK                    (0x40U)
#define USB_OBSERVE_DPPD_SHIFT                   (6U)
#define USB_OBSERVE_DPPD(x)                      (((cyg_uint8)(((cyg_uint8)(x)) << USB_OBSERVE_DPPD_SHIFT)) & USB_OBSERVE_DPPD_MASK)
#define USB_OBSERVE_DPPU_MASK                    (0x80U)
#define USB_OBSERVE_DPPU_SHIFT                   (7U)
#define USB_OBSERVE_DPPU(x)                      (((cyg_uint8)(((cyg_uint8)(x)) << USB_OBSERVE_DPPU_SHIFT)) & USB_OBSERVE_DPPU_MASK)

/*! @name CONTROL - USB OTG Control register */
#define USB_CONTROL_DPPULLUPNONOTG_MASK          (0x10U)
#define USB_CONTROL_DPPULLUPNONOTG_SHIFT         (4U)
#define USB_CONTROL_DPPULLUPNONOTG(x)            (((cyg_uint8)(((cyg_uint8)(x)) << USB_CONTROL_DPPULLUPNONOTG_SHIFT)) & USB_CONTROL_DPPULLUPNONOTG_MASK)

/*! @name USBTRC0 - USB Transceiver Control register 0 */
#define USB_USBTRC0_USB_RESUME_INT_MASK          (0x1U)
#define USB_USBTRC0_USB_RESUME_INT_SHIFT         (0U)
#define USB_USBTRC0_USB_RESUME_INT(x)            (((cyg_uint8)(((cyg_uint8)(x)) << USB_USBTRC0_USB_RESUME_INT_SHIFT)) & USB_USBTRC0_USB_RESUME_INT_MASK)
#define USB_USBTRC0_SYNC_DET_MASK                (0x2U)
#define USB_USBTRC0_SYNC_DET_SHIFT               (1U)
#define USB_USBTRC0_SYNC_DET(x)                  (((cyg_uint8)(((cyg_uint8)(x)) << USB_USBTRC0_SYNC_DET_SHIFT)) & USB_USBTRC0_SYNC_DET_MASK)
#define USB_USBTRC0_USB_CLK_RECOVERY_INT_MASK    (0x4U)
#define USB_USBTRC0_USB_CLK_RECOVERY_INT_SHIFT   (2U)
#define USB_USBTRC0_USB_CLK_RECOVERY_INT(x)      (((cyg_uint8)(((cyg_uint8)(x)) << USB_USBTRC0_USB_CLK_RECOVERY_INT_SHIFT)) & USB_USBTRC0_USB_CLK_RECOVERY_INT_MASK)
#define USB_USBTRC0_USBRESMEN_MASK               (0x20U)
#define USB_USBTRC0_USBRESMEN_SHIFT              (5U)
#define USB_USBTRC0_USBRESMEN(x)                 (((cyg_uint8)(((cyg_uint8)(x)) << USB_USBTRC0_USBRESMEN_SHIFT)) & USB_USBTRC0_USBRESMEN_MASK)
#define USB_USBTRC0_USBRESET_MASK                (0x80U)
#define USB_USBTRC0_USBRESET_SHIFT               (7U)
#define USB_USBTRC0_USBRESET(x)                  (((cyg_uint8)(((cyg_uint8)(x)) << USB_USBTRC0_USBRESET_SHIFT)) & USB_USBTRC0_USBRESET_MASK)

/*! @name USBFRMADJUST - Frame Adjust Register */
#define USB_USBFRMADJUST_ADJ_MASK                (0xFFU)
#define USB_USBFRMADJUST_ADJ_SHIFT               (0U)
#define USB_USBFRMADJUST_ADJ(x)                  (((cyg_uint8)(((cyg_uint8)(x)) << USB_USBFRMADJUST_ADJ_SHIFT)) & USB_USBFRMADJUST_ADJ_MASK)

/*! @name CLK_RECOVER_CTRL - USB Clock recovery control */
#define USB_CLK_RECOVER_CTRL_RESTART_IFRTRIM_EN_MASK (0x20U)
#define USB_CLK_RECOVER_CTRL_RESTART_IFRTRIM_EN_SHIFT (5U)
#define USB_CLK_RECOVER_CTRL_RESTART_IFRTRIM_EN(x) (((cyg_uint8)(((cyg_uint8)(x)) << USB_CLK_RECOVER_CTRL_RESTART_IFRTRIM_EN_SHIFT)) & USB_CLK_RECOVER_CTRL_RESTART_IFRTRIM_EN_MASK)
#define USB_CLK_RECOVER_CTRL_RESET_RESUME_ROUGH_EN_MASK (0x40U)
#define USB_CLK_RECOVER_CTRL_RESET_RESUME_ROUGH_EN_SHIFT (6U)
#define USB_CLK_RECOVER_CTRL_RESET_RESUME_ROUGH_EN(x) (((cyg_uint8)(((cyg_uint8)(x)) << USB_CLK_RECOVER_CTRL_RESET_RESUME_ROUGH_EN_SHIFT)) & USB_CLK_RECOVER_CTRL_RESET_RESUME_ROUGH_EN_MASK)
#define USB_CLK_RECOVER_CTRL_CLOCK_RECOVER_EN_MASK (0x80U)
#define USB_CLK_RECOVER_CTRL_CLOCK_RECOVER_EN_SHIFT (7U)
#define USB_CLK_RECOVER_CTRL_CLOCK_RECOVER_EN(x) (((cyg_uint8)(((cyg_uint8)(x)) << USB_CLK_RECOVER_CTRL_CLOCK_RECOVER_EN_SHIFT)) & USB_CLK_RECOVER_CTRL_CLOCK_RECOVER_EN_MASK)

/*! @name CLK_RECOVER_IRC_EN - IRC48M oscillator enable register */
#define USB_CLK_RECOVER_IRC_EN_REG_EN_MASK       (0x1U)
#define USB_CLK_RECOVER_IRC_EN_REG_EN_SHIFT      (0U)
#define USB_CLK_RECOVER_IRC_EN_REG_EN(x)         (((cyg_uint8)(((cyg_uint8)(x)) << USB_CLK_RECOVER_IRC_EN_REG_EN_SHIFT)) & USB_CLK_RECOVER_IRC_EN_REG_EN_MASK)
#define USB_CLK_RECOVER_IRC_EN_IRC_EN_MASK       (0x2U)
#define USB_CLK_RECOVER_IRC_EN_IRC_EN_SHIFT      (1U)
#define USB_CLK_RECOVER_IRC_EN_IRC_EN(x)         (((cyg_uint8)(((cyg_uint8)(x)) << USB_CLK_RECOVER_IRC_EN_IRC_EN_SHIFT)) & USB_CLK_RECOVER_IRC_EN_IRC_EN_MASK)

/*! @name CLK_RECOVER_INT_STATUS - Clock recovery separated interrupt status */
#define USB_CLK_RECOVER_INT_STATUS_OVF_ERROR_MASK (0x10U)
#define USB_CLK_RECOVER_INT_STATUS_OVF_ERROR_SHIFT (4U)
#define USB_CLK_RECOVER_INT_STATUS_OVF_ERROR(x)  (((cyg_uint8)(((cyg_uint8)(x)) << USB_CLK_RECOVER_INT_STATUS_OVF_ERROR_SHIFT)) & USB_CLK_RECOVER_INT_STATUS_OVF_ERROR_MASK)


/*!
 * @}
 */ /* end of group USB_Register_Masks */


/* USB - Peripheral instance base addresses */
/** Peripheral USB0 base address */
#define USB0_BASE                                (0x40072000u)
/** Peripheral USB0 base pointer */
#define USB0                                     ((USB_Type *)USB0_BASE)
/** Array initializer of USB peripheral base addresses */
#define USB_BASE_ADDRS                           { USB0_BASE }
/** Array initializer of USB peripheral base pointers */
#define USB_BASE_PTRS                            { USB0 }
/** Interrupt vectors for the USB peripheral type */
#define USB_IRQS                                 { USB0_IRQn }


/*!
 * @addtogroup usb_device_controller_khci_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief The maximum value of ISO maximum packet size for FS in USB specification 2.0 */
#define USB_DEVICE_MAX_FS_ISO_MAX_PACKET_SIZE (1023U)

/*! @brief The maximum value of non-ISO maximum packet size for FS in USB specification 2.0 */
#define USB_DEVICE_MAX_FS_NONE_ISO_MAX_PACKET_SIZE (64U)

/*! @brief Set BDT buffer address */
#define USB_KHCI_BDT_SET_ADDRESS(bdt_base, ep, direction, odd, address)                          \
    *((volatile cyg_uint32 *)((bdt_base & 0xfffffe00U) | (((cyg_uint32)ep & 0x0fU) << 5U) |          \
                            (((cyg_uint32)direction & 1U) << 4U) | (((cyg_uint32)odd & 1U) << 3U)) + \
      1U) = address

/*! @brief Set BDT control fields*/
#define USB_KHCI_BDT_SET_CONTROL(bdt_base, ep, direction, odd, control)                \
    *(volatile cyg_uint32 *)((bdt_base & 0xfffffe00U) | (((cyg_uint32)ep & 0x0fU) << 5U) | \
                           (((cyg_uint32)direction & 1U) << 4U) | (((cyg_uint32)odd & 1U) << 3U)) = control

/*! @brief Get BDT buffer address*/
#define USB_KHCI_BDT_GET_ADDRESS(bdt_base, ep, direction, odd)                                    \
    (*((volatile cyg_uint32 *)((bdt_base & 0xfffffe00U) | (((cyg_uint32)ep & 0x0fU) << 5U) |          \
                             (((cyg_uint32)direction & 1U) << 4U) | (((cyg_uint32)odd & 1U) << 3U)) + \
       1U))

/*! @brief Get BDT control fields*/
#define USB_KHCI_BDT_GET_CONTROL(bdt_base, ep, direction, odd)                          \
    (*(volatile cyg_uint32 *)((bdt_base & 0xfffffe00U) | (((cyg_uint32)ep & 0x0fU) << 5U) | \
                            (((cyg_uint32)direction & 1U) << 4U) | (((cyg_uint32)odd & 1U) << 3U)))


/*! @brief Endpoint state structure */
typedef struct _usb_device_khci_endpoint_state_struct
{
    cyg_uint8 *transferBuffer; /*!< Address of buffer containing the data to be transmitted */
    cyg_uint32 transferLength; /*!< Length of data to transmit. */
    cyg_uint32 transferDone;   /*!< The data length has been transferred*/
    union
    {
        cyg_uint32 state; /*!< The state of the endpoint */
        struct
        {
            cyg_uint32 maxPacketSize : 10U; /*!< The maximum packet size of the endpoint */
            cyg_uint32 stalled : 1U;        /*!< The endpoint is stalled or not */
            cyg_uint32 data0 : 1U;          /*!< The data toggle of the transaction */
            cyg_uint32 bdtOdd : 1U;         /*!< The BDT toggle of the endpoint */
            cyg_uint32 dmaAlign : 1U;       /*!< Whether the transferBuffer is DMA aligned or not */
            cyg_uint32 transferring : 1U;   /*!< The endpoint is transferring */
            cyg_uint32 zlt : 1U;            /*!< zlt flag */
        } stateBitField;
    } stateUnion;
} usb_device_khci_endpoint_state_struct_t;

/*! @brief KHCI state structure */
typedef struct _usb_device_khci_state_struct
{
#if 0
    usb_device_struct_t *deviceHandle; /*!< Device handle used to identify the device object belongs to */
#endif
    cyg_uint8 *bdt;                      /*!< BDT buffer address */
    volatile USB_Type *registerBase;            /*!< The base address of the register */
    cyg_uint8 setupPacketBuffer[USB_SETUP_PACKET_SIZE * 2]; /*!< The setup request buffer */
    cyg_uint8 *dmaAlignBuffer; /*!< This buffer is used to fix the transferBuffer or transferLength does
                               not align to 4-bytes when the function USB_DeviceKhciRecv is called.
                               The macro USB_DEVICE_CONFIG_KHCI_DMA_ALIGN is used to enable or disable this feature.
                               If the feature is enabled, when the transferBuffer or transferLength does not align to
                               4-bytes,
                               the transferLength is not more than USB_DEVICE_CONFIG_KHCI_DMA_ALIGN_BUFFER_LENGTH, and
                               the flag isDmaAlignBufferInusing is zero, the dmaAlignBuffer is used to receive data
                               and the flag isDmaAlignBufferInusing is set to 1.
                               When the transfer is done, the received data, kept in dmaAlignBuffer, is copied
                               to the transferBuffer, and the flag isDmaAlignBufferInusing is cleared.
                                */
    usb_device_khci_endpoint_state_struct_t
        endpointState[CYGNUM_DEVS_USB_KINETIS_CONFIG_ENDPOINTS * 2]; /*!< Endpoint state structures */
    cyg_uint8 isDmaAlignBufferInusing;                    /*!< The dmaAlignBuffer is used or not */
    cyg_uint8 isResetting;                                /*!< Is doing device reset or not */
    cyg_uint8 controllerId;                               /*!< Controller ID */
    cyg_uint8 setupBufferIndex;                           /*!< A valid setup buffer flag */
#if (defined(USB_DEVICE_CONFIG_OTG) && (USB_DEVICE_CONFIG_OTG))
    cyg_uint8 otgStatus;
#endif
} __attribute__((__aligned__(4))) usb_device_khci_state_struct_t;

/*! @brief USB error code */
typedef enum _usb_status
{
    kStatus_USB_Success = 0x00U, /*!< Success */
    kStatus_USB_Error,           /*!< Failed */

    kStatus_USB_Busy,                       /*!< Busy */
    kStatus_USB_InvalidHandle,              /*!< Invalid handle */
    kStatus_USB_InvalidParameter,           /*!< Invalid parameter */
    kStatus_USB_InvalidRequest,             /*!< Invalid request */
    kStatus_USB_ControllerNotFound,         /*!< Controller cannot be found */
    kStatus_USB_InvalidControllerInterface, /*!< Invalid controller interface */

    kStatus_USB_NotSupported,   /*!< Configuration is not supported */
    kStatus_USB_Retry,          /*!< Enumeration get configuration retry */
    kStatus_USB_TransferStall,  /*!< Transfer stalled */
    kStatus_USB_TransferFailed, /*!< Transfer failed */
    kStatus_USB_AllocFail,      /*!< Allocation failed */
    kStatus_USB_LackSwapBuffer, /*!< Insufficient swap buffer for KHCI */
    kStatus_USB_TransferCancel, /*!< The transfer cancelled */
    kStatus_USB_BandwidthFail,  /*!< Allocate bandwidth failed */
    kStatus_USB_MSDStatusFail,  /*!< For MSD, the CSW status means fail */
    kStatus_USB_EHCIAttached,
    kStatus_USB_EHCIDetached,
} usb_status_t;

/*! @brief Endpoint initialization structure */
typedef struct _usb_device_endpoint_init_struct
{
    cyg_uint16 maxPacketSize;  /*!< Endpoint maximum packet size */
    cyg_uint8 endpointAddress; /*!< Endpoint address*/
    cyg_uint8 transferType;    /*!< Endpoint transfer type*/
    cyg_uint8 zlt;             /*!< ZLT flag*/
} usb_device_endpoint_init_struct_t;

/*! @brief Endpoint status structure */
typedef struct _usb_device_endpoint_status_struct
{
    cyg_uint8  endpointAddress; /*!< Endpoint address */
    cyg_uint16 endpointStatus; /*!< Endpoint status : idle or stalled */
} usb_device_endpoint_status_struct_t;

/*! @brief Defines endpoint state */
typedef enum _usb_endpoint_status
{
    kUSB_DeviceEndpointStateIdle = 0U, /*!< Endpoint state, idle*/
    kUSB_DeviceEndpointStateStalled,   /*!< Endpoint state, stalled*/
} usb_device_endpoint_status_t;

#define USB_KHCI_BDT_DEVICE_OUT_TOKEN (0x01U)
#define USB_KHCI_BDT_DEVICE_IN_TOKEN (0x09U)
#define USB_KHCI_BDT_DEVICE_SETUP_TOKEN (0x0DU)

#define USB_KHCI_BDT_OWN (0x80U)
#define USB_KHCI_BDT_DATA01(x) ((((cyg_uint32)(x)) & 0x01U) << 0x06U)
#define USB_KHCI_BDT_BC(x) ((((cyg_uint32)(x)) & 0x3FFU) << 0x10U)
#define UBS_KHCI_BDT_KEEP (0x20U)
#define UBS_KHCI_BDT_NINC (0x10U)
#define USB_KHCI_BDT_DTS (0x08U)
#define USB_KHCI_BDT_STALL (0x04U)

typedef enum _usb_khci_interrupt_type
{
    kUSB_KhciInterruptReset = 0x01U,
    kUSB_KhciInterruptError = 0x02U,
    kUSB_KhciInterruptSofToken = 0x04U,
    kUSB_KhciInterruptTokenDone = 0x08U,
    kUSB_KhciInterruptSleep = 0x10U,
    kUSB_KhciInterruptResume = 0x20U,
    kUSB_KhciInterruptAttach = 0x40U,
    kUSB_KhciInterruptStall = 0x80U,
} usb_khci_interrupt_type_t;

/* USB standard descriptor endpoint bmAttributes */
#define USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK (0x80U)
#define USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_SHIFT (7U)
#define USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_OUT (0U)
#define USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_IN (0x80U)

/*! @brief Control type for controller */
typedef enum _usb_device_control_type
{
    kUSB_DeviceControlRun = 0U,          /*!< Enable the device functionality */
    kUSB_DeviceControlStop,              /*!< Disable the device functionality */
    kUSB_DeviceControlEndpointInit,      /*!< Initialize a specified endpoint */
    kUSB_DeviceControlEndpointDeinit,    /*!< De-initialize a specified endpoint */
    kUSB_DeviceControlEndpointStall,     /*!< Stall a specified endpoint */
    kUSB_DeviceControlEndpointUnstall,   /*!< Unstall a specified endpoint */
    kUSB_DeviceControlGetDeviceStatus,   /*!< Get device status */
    kUSB_DeviceControlGetEndpointStatus, /*!< Get endpoint status */
    kUSB_DeviceControlSetDeviceAddress,  /*!< Set device address */
    kUSB_DeviceControlGetSynchFrame,     /*!< Get current frame */
    kUSB_DeviceControlResume,            /*!< Drive controller to generate a resume signal in USB bus */
    kUSB_DeviceControlSleepResume,       /*!< Drive controller to generate a LPM resume signal in USB bus */
    kUSB_DeviceControlSuspend,           /*!< Drive controller to enetr into suspend mode */
    kUSB_DeviceControlSleep,             /*!< Drive controller to enetr into sleep mode */
    kUSB_DeviceControlSetDefaultStatus,  /*!< Set controller to default status */
    kUSB_DeviceControlGetSpeed,          /*!< Get current speed */
    kUSB_DeviceControlGetOtgStatus,      /*!< Get OTG status */
    kUSB_DeviceControlSetOtgStatus,      /*!< Set OTG status */
    kUSB_DeviceControlSetTestMode,       /*!< Drive xCHI into test mode */
    kUSB_DeviceControlGetRemoteWakeUp,   /*!< Get flag of LPM Remote Wake-up Enabled by USB host. */
#if (defined(USB_DEVICE_CHARGER_DETECT_ENABLE) && (USB_DEVICE_CHARGER_DETECT_ENABLE > 0U))
    kUSB_DeviceControlDcdInitModule,
    kUSB_DeviceControlDcdDeinitModule,
    kUSB_DeviceControlGetDeviceAttachStatus,
#endif
} usb_device_control_type_t;
#endif /* USB_HDR_H */

/* USB  standard descriptor transfer direction (cannot change the value because iTD use the value directly) */
#define USB_OUT (0U)
#define USB_IN (1U)

/*! @brief The setup packet size of USB control transfer. */
#define USB_SETUP_PACKET_SIZE (8U)
/*! @brief  USB endpoint mask */
#define USB_ENDPOINT_NUMBER_MASK (0x0FU)

#define USB_DEVICE_CONFIG_KHCI_DMA_ALIGN_BUFFER_LENGTH  8

/* USB speed (the value cannot be changed because EHCI QH use the value directly)*/
#define USB_SPEED_FULL (0x00U)
#define USB_SPEED_LOW (0x01U)
#define USB_SPEED_HIGH (0x02U)

/* Set up packet structure */
typedef struct _usb_setup_struct
{
    cyg_uint8  bmRequestType;
    cyg_uint8  bRequest;
    cyg_uint16 wValue;
    cyg_uint16 wIndex;
    cyg_uint16 wLength;
} usb_setup_struct_t;



#endif