"""Test actual TELEM1 queue/formatter C with an asynchronous DMA fake."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
GCC = r"D:\Users\21144\w64devkit\bin\gcc.exe"
HAL = r'''
#ifndef STM32F4XX_HAL_H
#define STM32F4XX_HAL_H
#include <stdint.h>
#define __NVIC_PRIO_BITS 4U
#define DMA_SxCR_EN 1U
typedef struct { uint32_t CR; } DMA_Stream_TypeDef;
typedef struct { DMA_Stream_TypeDef *Instance; } DMA_HandleTypeDef;
typedef struct { uint32_t SR; } USART_TypeDef;
typedef struct { DMA_HandleTypeDef *hdmatx; uint32_t ErrorCode; struct { uint32_t BaudRate; } Init; } UART_HandleTypeDef;
typedef enum { HAL_OK=0, HAL_ERROR=1, HAL_BUSY=2 } HAL_StatusTypeDef;
static inline uint32_t __get_BASEPRI(void) { return 0U; }
static inline void __set_BASEPRI_MAX(uint32_t x) { (void)x; }
static inline void __set_BASEPRI(uint32_t x) { (void)x; }
static inline void __DSB(void) {}
static inline void __ISB(void) {}
static inline void __DMB(void) {}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_UART_AbortTransmit_IT(UART_HandleTypeDef *);
#endif
'''
HARNESS = r'''
#include "imu_telemetry.h"
#include "board_link_telem.h"
#include "board_link_app.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static DMA_Stream_TypeDef stream;
static DMA_HandleTypeDef dma = {&stream};
static UART_HandleTypeDef uart = {&dma, 0, {460800}};
static uint8_t *owned;
static uint16_t owned_length;
static char saved[IMU_TELEMETRY_TX_BUFFER_SIZE];
static board_link_message_handler_t observer;
void BoardLinkApp_SetMessageHandler(board_link_message_handler_t callback) { observer = callback; }
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *u, uint8_t *p, uint16_t n) {
    assert(u == &uart && owned == NULL && n < sizeof(saved));
    owned=p; owned_length=n; memcpy(saved,p,n); saved[n]=0;
    stream.CR = DMA_SxCR_EN;
    return HAL_OK;
}
static void complete(void) {
    assert(owned != NULL && memcmp(owned,saved,owned_length)==0);
    owned=NULL; stream.CR=0;
    ImuTelemetry_OnTxComplete(&uart);
}
HAL_StatusTypeDef HAL_UART_AbortTransmit_IT(UART_HandleTypeDef *u) {
    assert(u==&uart && owned!=NULL && memcmp(owned,saved,owned_length)==0);
    owned=NULL; stream.CR=0;
    ImuTelemetry_OnAbortComplete(u); return HAL_OK;
}
int main(void) {
    ImuTelemetryRecord record = {0};
    ImuTelemetryStats stats;
    bl_message_t message = {0};
    char line[IMU_TELEMETRY_TX_BUFFER_SIZE];
    size_t length;
    unsigned i;
    ImuTelemetry_Init(&uart,0);
#if IMU_TELEMETRY_OUTPUT_FORMAT == IMU_TELEMETRY_FORMAT_SEEKFREE && IMU_TELEMETRY_VIEW != IMU_TELEMETRY_VIEW_RC
    BoardLinkTelem_Init(); assert(observer!=NULL);
    message.source=1; message.kind=BL_KIND_EVENT; message.command=0x0200; message.length=60;
    message.payload[0]=1; message.payload[1]=1; message.payload[2]=3; message.payload[3]=10;
    observer(&message); ImuTelemetry_Service(NULL,1); assert(owned==NULL);
    assert(g_board_link_telem_status.queued==0);
    record.flags=IMU_TELEMETRY_FLAG_SAMPLE_VALID; record.timestamp_us=10000;
    record.gyro_raw[0]=-164; record.gyro_raw[1]=164; record.gyro_raw[2]=0;
    record.accel_raw[0]=-2048; record.accel_raw[1]=2048; record.accel_raw[2]=0;
    record.attitude_present=1; record.attitude_flags=1; record.attitude_timestamp_us=10000;
    record.attitude_mdeg[0]=-12345; record.attitude_mdeg[1]=0; record.attitude_mdeg[2]=179999;
    record.heading_valid=1;
    record.mag_present=1; record.mag_healthy=1; record.mag_calibrated=1;
    record.mag_raw_mut[0]=-35600; record.mag_raw_mut[1]=12008; record.mag_raw_mut[2]=49000;
    record.baro_pressure_pa=99839; record.baro_temperature_centi_c=2534;
    record.baro_relative_mm=-123; record.baro_filtered_mm=-120;
    record.baro_flags=7; record.baro_model=5611; record.baro_state=2;
    ImuTelemetry_Service(&record,10000); assert(owned!=NULL);
#if IMU_TELEMETRY_VIEW == IMU_TELEMETRY_VIEW_GYRO
    assert(!strcmp(saved,"gyro:-10.000,10.000,0.000\n"));
#elif IMU_TELEMETRY_VIEW == IMU_TELEMETRY_VIEW_ACCEL
    assert(!strcmp(saved,"accel:-1.000,1.000,0.000\n"));
#elif IMU_TELEMETRY_VIEW == IMU_TELEMETRY_VIEW_ATTITUDE
    assert(!strcmp(saved,"attitude:-12.345,0.000,179.999,1\n"));
#elif IMU_TELEMETRY_VIEW == IMU_TELEMETRY_VIEW_MAG
    assert(!strcmp(saved,"mag:-35.600,12.008,49.000,1\n"));
#else
    assert(!strcmp(saved,"baro:99839,2534,-123,-120,7,5611,2,0\n"));
#endif
    { unsigned commas=0;
      assert(owned_length==strlen(saved) && saved[owned_length-1]=='\n');
      for(i=0;i+1U<owned_length;i++) {
          if(saved[i]==',') { commas++; }
          assert(saved[i]!='\n' && saved[i]!='\r');
      }
      assert(commas<8);
    }
    /* Changing producer data while DMA is active cannot alter its frame. */
    record.gyro_raw[0]=-32768; record.gyro_raw[1]=32767; record.gyro_raw[2]=1;
    ImuTelemetry_Service(&record,10001); complete();
    record.timestamp_us=20000; record.attitude_timestamp_us=20000;
    ImuTelemetry_Service(&record,20000); assert(owned!=NULL);
#if IMU_TELEMETRY_VIEW == IMU_TELEMETRY_VIEW_GYRO
    assert(!strcmp(saved,"gyro:-1998.049,1997.988,0.061\n"));
#endif
    complete();
    record.flags=0; record.attitude_flags=0; record.mag_healthy=0; record.baro_flags=0;
    ImuTelemetry_Service(&record,30000);
#if IMU_TELEMETRY_VIEW == IMU_TELEMETRY_VIEW_BARO
    assert(strstr(saved,",0,5611,2,0\n")); complete();
#else
    assert(owned==NULL); /* No startup, stale or fault diagnostic text mixed in. */
#endif
    puts("PASS: SeekFree sensor preset, units, LF, channel limit, validity, RC silence and DMA ownership");
    return 0;
#elif BARO_TELEMETRY_ENABLED
    BoardLinkTelem_Init(); assert(observer!=NULL);
    message.source=1; message.kind=BL_KIND_EVENT; message.command=0x0200; message.length=60;
    message.payload[0]=1; message.payload[1]=1; message.payload[2]=3; message.payload[3]=10;
    observer(&message); ImuTelemetry_Service(&record,1); assert(owned==NULL);
    strcpy(line,"baro:100000,200000,0,2000,0,0,1,0,2,0,1,0,0,0\n");
    assert(ImuTelemetry_QueueText(line,(uint16_t)strlen(line)));
    ImuTelemetry_Service(&record,2); assert(strstr(saved,"baro:")==saved); complete();
    puts("PASS: barometer console uses shared DMA and suppresses RC channel mixing");
    return 0;
#elif IMU_TELEMETRY_RC_ONLY
    BoardLinkTelem_Init(); assert(observer!=NULL);
    record.flags=IMU_TELEMETRY_FLAG_SAMPLE_VALID;
    for(i=0;i<100;i++) { ImuTelemetry_Service(&record,i*10000U); assert(owned==NULL); }
    message.source=1; message.kind=BL_KIND_REQUEST; message.command=BL_CMD_PING;
    observer(&message); ImuTelemetry_Service(&record,1000000); assert(owned==NULL);
    message.kind=BL_KIND_EVENT; message.command=0x0200; message.length=60;
    message.payload[0]=1; message.payload[1]=1; message.payload[2]=3; message.payload[3]=10;
    for(i=0;i<10;i++) { unsigned value=200+i*100; message.payload[40+i*2]=value&255; message.payload[41+i*2]=value>>8; }
    observer(&message); ImuTelemetry_Service(&record,1000001);
#ifdef TEST_RC_STATUS_FIELDS
    assert(!strcmp(saved,"rc:200,300,400,500,1100,1,1,0\n"));
#else
    assert(!strcmp(saved,"rc:200,300,400,500,600,700,800,900\n"));
#endif
    { unsigned commas=0;
      assert(owned_length==strlen(saved) && saved[owned_length-1]=='\n');
      for(i=0;i+1U<owned_length;i++) {
          if(saved[i]==',') { commas++; }
          assert(saved[i]!='\n' && saved[i]!='\r');
      }
      assert(commas==7);
    }
    message.payload[1]=3; observer(&message); ImuTelemetry_Service(&record,1000002);
    /* A queued failsafe sample cannot mutate the active valid DMA frame. */
    assert(memcmp(owned,saved,owned_length)==0); complete();
    ImuTelemetry_Service(&record,1000003);
#ifdef TEST_RC_STATUS_FIELDS
    assert(!strcmp(saved,"rc:200,300,400,500,1100,0,3,0\n"));
#else
    assert(!strcmp(saved,"rc:200,300,400,500,600,700,800,900\n"));
#endif
    complete();
    message.payload[1]=1; message.payload[8]=100;
    observer(&message); ImuTelemetry_Service(&record,1000004);
#ifdef TEST_RC_STATUS_FIELDS
    assert(!strcmp(saved,"rc:200,300,400,500,1100,0,1,100\n"));
#else
    assert(!strcmp(saved,"rc:200,300,400,500,600,700,800,900\n"));
#endif
    complete();
    message.payload[8]=0; message.payload[40]=255; message.payload[41]=255;
    observer(&message); ImuTelemetry_Service(&record,1000005); assert(owned==NULL);
    assert(!BoardLinkTelem_Format(&message,line,sizeof(line)));
    message.payload[40]=200; message.payload[41]=0;
    assert(!BoardLinkTelem_Format(&message,line,8));
    message.length=59; assert(!BoardLinkTelem_Format(&message,line,sizeof(line)));
    message.length=60; message.source=4; observer(&message);
    ImuTelemetry_Service(&record,1000006); assert(owned==NULL);
    ImuTelemetry_GetStats(&stats); assert(stats.frames_started==3 && stats.attitude_frames==0 && stats.status_frames==0);
    puts("PASS: SeekFree Printf 8-channel format, selection, RC validation, silence and DMA ownership");
    return 0;
#elif IMU_TELEMETRY_OUTPUT_FORMAT == IMU_TELEMETRY_FORMAT_BINARY
    assert(!ImuTelemetry_QueueText("#LINK\n",6));
    ImuTelemetry_Service(&record,10000);
    assert(owned_length==64 && (uint8_t)saved[0]==0xa5 && (uint8_t)saved[1]==0x5a);
    complete(); puts("PASS: binary telemetry rejects auxiliary text and retains 64-byte packets");
    return 0;
#else
    BoardLinkTelem_Init(); assert(observer!=NULL);
    message.source=1; message.destination=4; message.kind=BL_KIND_REQUEST;
    message.command=BL_CMD_PING; message.sequence=42; message.length=23;
    message.payload[0]=99; message.payload[4]=0xe8; message.payload[5]=3;
    memcpy(message.payload+8,"F1->F4 link test",15);
    observer(&message);
    ImuTelemetry_Service(NULL,1);
    assert(strstr(saved,"#LINK_RX src=F1 type=REQ cmd=0x0001 seq=42 len=23")!=NULL);
    assert(strstr(saved,"count=99 sender_ms=1000")!=NULL);
    /* Existing test packets carry fifteen text bytes; display bytes verbatim. */
    assert(strstr(saved,"text=\"F1->F4 link tes\"")!=NULL);
    message.sequence=43; observer(&message);
    ImuTelemetry_Service(NULL,2); /* Cannot overwrite the first live DMA buffer. */
    assert(strstr(saved,"seq=42")!=NULL); complete();
    ImuTelemetry_Service(NULL,3); assert(strstr(saved,"seq=43")!=NULL); complete();
    record.flags=IMU_TELEMETRY_FLAG_SAMPLE_VALID;
    record.accel_raw[0]=123;
    ImuTelemetry_Service(&record,10000); assert(strstr(saved,"mpu6000:123,")==saved);
    observer(&message); ImuTelemetry_Service(&record,10001); complete();
    ImuTelemetry_Service(&record,10002); assert(strstr(saved,"#LINK_RX")==saved); complete();
    for(i=0;i<5;i++) { message.sequence=(uint16_t)(100+i); observer(&message); }
    for(i=0;i<4;i++) { ImuTelemetry_Service(NULL,10003+i); complete(); }
    ImuTelemetry_GetStats(&stats);
    assert(stats.text_queued==7 && stats.text_started==7 && stats.text_dropped==1);
    assert(g_board_link_telem_status.queued==7 && g_board_link_telem_status.dropped==1);
    observer(&message);
    ImuTelemetry_Service(NULL,19999); assert(owned==NULL); /* Protect next IMU slot. */
    ImuTelemetry_Service(&record,20000); assert(strstr(saved,"mpu6000:")==saved); complete();
    ImuTelemetry_Service(NULL,20001); assert(strstr(saved,"#LINK_RX")==saved); complete();
    message.command=0x0100; message.sequence=65535; message.length=64;
    for(i=0;i<64;i++) { message.payload[i]=(uint8_t)i; }
    length=BoardLinkTelem_Format(&message,line,sizeof(line));
    assert(length>0 && length<sizeof(line) && line[length-1]=='\n');
    assert(strlen(strstr(line," hex=")+5)==129); /* 128 hex digits + LF */
    for(i=0;i<length-1;i++) { assert(line[i]!='\n' && line[i]!='\r'); }
    assert(!BoardLinkTelem_Format(&message,line,16));
    assert(!BoardLinkTelem_Format(NULL,line,sizeof(line)));
    message.length=65; assert(!BoardLinkTelem_Format(&message,line,sizeof(line)));
    message.length=64; observer(&message); ImuTelemetry_Service(NULL,20007);
    ImuTelemetry_Service(NULL,41000); /* Timeout must release via confirmed abort. */
    ImuTelemetry_GetStats(&stats); assert(stats.tx_timeouts==1 && !stats.faulted);
    puts("PASS: real TELEM1 formatter, maximum payload, safe text, async DMA ownership, queue overflow and timeout recovery");
    return 0;
#endif
}
'''

with tempfile.TemporaryDirectory() as temporary:
    temp = Path(temporary)
    (temp / "stm32f4xx_hal.h").write_text(HAL)
    (temp / "harness.c").write_text(HARNESS)
    configurations = [(1, 0, 0, 0, 0), (0, 0, 0, 0, 0), (1, 1, 0, 0, 0),
                      (1, 1, 1, 0, 0), (1, 1, 0, 1, 0)]
    configurations += [(2, 1, 0, 0, view) for view in range(6)]
    # Check the shipped header defaults and the dedicated barometer target,
    # without overriding the view: these are the actual Keil selections.
    configurations += [(2, 1, 0, 0, None), (2, 1, 1, 0, None)]
    for mode, rc_only, baro, selected, view in configurations:
        exe = temp / f"test_{mode}_{rc_only}_{baro}_{selected}_{view}.exe"
        # Binary mode intentionally excludes text-only harness variables.
        command = [GCC, "-std=c99", "-Wall", "-Wextra", "-Werror", "-Wno-unused-variable",
                   "-Wno-unused-function", f"-DIMU_TELEMETRY_RC_ONLY={rc_only}", f"-DIMU_TELEMETRY_OUTPUT_FORMAT={mode}",
                   f"-DBARO_TELEMETRY_ENABLED={baro}",
                   "-I" + str(temp), "-I" + str(ROOT / "Core/Inc"),
                   "-I" + str(ROOT.parents[1] / "firmware/common"),
                   str(temp / "harness.c"), str(ROOT / "Core/Src/board_link_telem.c"),
                   str(ROOT / "Core/Src/imu_telemetry.c"), "-o", str(exe)]
        if view is not None:
            command.insert(1, f"-DIMU_TELEMETRY_VIEW={view}")
        if selected:
            command[1:1] = ["-DTEST_RC_STATUS_FIELDS=1", "-DBOARD_LINK_TELEM_RC_FIELDS=1U,2U,3U,4U,10U,11U,12U,13U"]
        subprocess.run(command, check=True)
        subprocess.run([str(exe)], check=True)
    command = [arg for arg in command if not arg.startswith("-DBOARD_LINK_TELEM_RC_FIELDS=")]
    command.insert(1, "-DBOARD_LINK_TELEM_RC_FIELDS=1U,2U,3U,4U,5U,6U,7U,8U,9U")
    rejected = subprocess.run(command, capture_output=True, text=True)
    assert rejected.returncode != 0 and "RcFieldCountCheck" in rejected.stderr
    print("PASS: compiler rejects configurations exceeding SeekFree's 8-channel limit")
