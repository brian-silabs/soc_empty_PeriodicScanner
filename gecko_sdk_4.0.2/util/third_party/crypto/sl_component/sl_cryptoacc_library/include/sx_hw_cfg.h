/**
 * @file
 * @brief Defines macros to be used to retrieve the hardware configuration
 * of CryptoMaster, i.e. what IPs are included.
 * @copyright Copyright (c) 2016-2018 Silex Insight. All Rights reserved
 */

#ifndef SX_HW_CFG_H
#define SX_HW_CFG_H
#include "cryptolib_def.h"

#define CRYPTOSOC_INCL_IPS_HW_CFG                      (*((const volatile uint32_t*) &CRYPTOACC->INCL_IPS_HW_CFG))

#define CRYPTOSOC_HW_CFG_AES_IP_INCLUDED_LSB           0
#define CRYPTOSOC_HW_CFG_AES_IP_INCLUDED_MASK          (1L<<CRYPTOSOC_HW_CFG_AES_IP_INCLUDED_LSB)
#define CRYPTOSOC_HW_CFG_HP_AESGCM_IP_INCLUDED_LSB     1
#define CRYPTOSOC_HW_CFG_HP_AESGCM_IP_INCLUDED_MASK    (1L<<CRYPTOSOC_HW_CFG_HP_AESGCM_IP_INCLUDED_LSB)
#define CRYPTOSOC_HW_CFG_HP_AESXTS_IP_INCLUDED_LSB     2
#define CRYPTOSOC_HW_CFG_HP_AESXTS_IP_INCLUDED_MASK    (1L<<CRYPTOSOC_HW_CFG_HP_AESXTS_IP_INCLUDED_LSB)
#define CRYPTOSOC_HW_CFG_DES_IP_INCLUDED_LSB           3
#define CRYPTOSOC_HW_CFG_DES_IP_INCLUDED_MASK          (1L<<CRYPTOSOC_HW_CFG_DES_IP_INCLUDED_LSB)
#define CRYPTOSOC_HW_CFG_HASH_IP_INCLUDED_LSB          4
#define CRYPTOSOC_HW_CFG_HASH_IP_INCLUDED_MASK         (1L<<CRYPTOSOC_HW_CFG_HASH_IP_INCLUDED_LSB)
#define CRYPTOSOC_HW_CFG_CHACHAPOLY_IP_INCLUDED_LSB    5
#define CRYPTOSOC_HW_CFG_CHACHAPOLY_IP_INCLUDED_MASK   (1L<<CRYPTOSOC_HW_CFG_CHACHAPOLY_IP_INCLUDED_LSB)
#define CRYPTOSOC_HW_CFG_SHA3_IP_INCLUDED_LSB          6
#define CRYPTOSOC_HW_CFG_SHA3_IP_INCLUDED_MASK         (1L<<CRYPTOSOC_HW_CFG_SHA3_IP_INCLUDED_LSB)
#define CRYPTOSOC_HW_CFG_ZUC_IP_INCLUDED_LSB           7
#define CRYPTOSOC_HW_CFG_ZUC_IP_INCLUDED_MASK          (1L<<CRYPTOSOC_HW_CFG_ZUC_IP_INCLUDED_LSB)
#define CRYPTOSOC_HW_CFG_SM4_IP_INCLUDED_LSB           8
#define CRYPTOSOC_HW_CFG_SM4_IP_INCLUDED_MASK          (1L<<CRYPTOSOC_HW_CFG_SM4_IP_INCLUDED_LSB)
#define CRYPTOSOC_HW_CFG_PKE_IP_INCLUDED_LSB           9
#define CRYPTOSOC_HW_CFG_PKE_IP_INCLUDED_MASK          (1L<<CRYPTOSOC_HW_CFG_PKE_IP_INCLUDED_LSB)
#define CRYPTOSOC_HW_CFG_NDRBG_IP_INCLUDED_LSB         10
#define CRYPTOSOC_HW_CFG_NDRBG_IP_INCLUDED_MASK        (1L<<CRYPTOSOC_HW_CFG_NDRBG_IP_INCLUDED_LSB)
#define CRYPTOSOC_HW_CFG_ARIA_IP_INCLUDED_LSB          14
#define CRYPTOSOC_HW_CFG_ARIA_IP_INCLUDED_MASK         (1L<<CRYPTOSOC_HW_CFG_ARIA_IP_INCLUDED_LSB)

#endif /* SX_HW_CFG_H */
