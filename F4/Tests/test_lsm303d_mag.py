"""Native tests of the real LSM303D driver, configuration and calibration gate."""
import ctypes as c
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile
import unittest
import zlib

ROOT=Path(__file__).resolve().parents[1]


class Status(c.Structure):
    _fields_=[(name,c.c_uint32) for name in ['state','error','identity','reg','expected','actual',
               'samples','not_ready','overruns','bus_errors','invalid','timestamp','max_interval','calibrated']]+[
               ('raw',c.c_int32*3),('field',c.c_float*3),('norm',c.c_float),
               ('sample_valid',c.c_uint32),('calibration_error',c.c_uint32)]


class Sample(c.Structure):
    _fields_=[('timestamp',c.c_uint32),('sequence',c.c_uint32),('flags',c.c_uint32),
              ('raw',c.c_float*3),('field',c.c_float*3),('reference',c.c_float)]


STUB=r'''
#include "lsm303d_mag.h"
#include <string.h>
static uint8_t registers[128];
static int busy, error, ignored;
static unsigned writes;
void stub_reset(void) { memset(registers,0,sizeof(registers)); registers[15]=0x49;
 registers[2]=0xa5; registers[0x15]=5; busy=error=writes=0; ignored=-1; }
void stub_set(int reg,int value) { registers[reg]=(uint8_t)value; }
int stub_read(int reg) { return registers[reg]; }
unsigned stub_writes(void) { return writes; }
void stub_busy(int value) { busy=value; }
void stub_error(int value) { error=value; }
void stub_ignore(int reg) { ignored=reg; }
void stub_data(int x,int y,int z,int status) {
 int values[3]={x,y,z}; unsigned i; registers[7]=(uint8_t)status;
 for(i=0;i<3;++i) { registers[8+2*i]=(uint8_t)values[i]; registers[9+2*i]=(uint8_t)((unsigned)values[i]>>8); }
}
int Mag_Port_Transfer(const uint8_t *tx,uint8_t *rx,uint16_t length) {
 unsigned i,reg=tx[0]&0x3f;
 if(busy) return 0; if(error) return -1;
 memset(rx,0,length);
 if(tx[0]&0x80) {
  for(i=1;i<length;++i) rx[i]=registers[reg+((tx[0]&0x40)?i-1:0)];
  if(reg==7 && length==8) registers[7]=0;
 } else { ++writes; if((int)reg!=ignored) registers[reg]=tx[1]; }
 return 1;
}
'''


def calibration_header(valid=False,corrupt=False):
    values=[1.,2.,4.,2.,0.,0.,0.,1.,0.,0.,0.,.5,50.]
    crc=zlib.crc32(struct.pack('<13f',*values)) ^ int(corrupt)
    return (f'#define MAG_CALIBRATION_VALID {int(valid)}U\n#define MAG_CALIBRATION_CRC32 0x{crc:08x}U\n'
            '#define MAG_CALIBRATION_REFERENCE_UT 50.0f\n#define MAG_CALIBRATION_BIAS {1.0f,2.0f,4.0f}\n'
            '#define MAG_CALIBRATION_MATRIX {{2.0f,0.0f,0.0f},{0.0f,1.0f,0.0f},{0.0f,0.0f,0.5f}}\n')


class MagTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory=tempfile.TemporaryDirectory()
        cls.libraries=[]
        compiler=shutil.which('gcc')
        if not compiler: raise RuntimeError('gcc required')
        for index,header in enumerate([calibration_header(),calibration_header(True),
                                       calibration_header(True,True),
                                       (ROOT/'Core/Inc/mag_calibration_params.h').read_text()]):
            folder=Path(cls.directory.name)/str(index); folder.mkdir()
            (folder/'mag_calibration_params.h').write_text(header)
            (folder/'stub.c').write_text(STUB)
            path=folder/'mag.dll'
            process=subprocess.run([compiler,'-std=c99','-Wall','-Wextra','-Werror','-Wno-misleading-indentation',
                                    '-shared','-O2','-I',str(folder),'-I',str(ROOT/'Core/Inc'),
                                    str(ROOT/'Core/Src/lsm303d_mag.c'),str(folder/'stub.c'),'-lm','-o',str(path)],
                                   capture_output=True,text=True,timeout=30)
            if process.returncode: raise AssertionError(process.stdout+process.stderr)
            lib=c.CDLL(str(path)); lib.Mag_GetStatus.argtypes=[c.POINTER(Status)]
            lib.Mag_PopSample.argtypes=[c.POINTER(Sample)]; lib.Mag_PopSample.restype=c.c_bool
            lib.Mag_Service.argtypes=[c.c_uint32,c.c_bool]
            cls.libraries.append(lib)

    @classmethod
    def tearDownClass(cls):
        import _ctypes
        for lib in cls.libraries: _ctypes.FreeLibrary(lib._handle)
        cls.directory.cleanup()

    def setUp(self):
        self.lib=self.libraries[0]; self.lib.stub_reset(); self.lib.Mag_Init(0)

    def status(self):
        result=Status(); self.lib.Mag_GetStatus(c.byref(result)); return result

    def initialize(self):
        for i in range(20): self.lib.Mag_Service(i*10,True)
        self.assertEqual((self.status().state,self.status().error),(2,0))

    def sample(self):
        sample=Sample(); self.assertTrue(self.lib.Mag_PopSample(c.byref(sample))); return sample

    def test_exact_identity_and_register_configuration(self):
        self.initialize()
        self.assertEqual([self.lib.stub_read(r) for r in [2,0x15,0x20,0x24,0x25,0x26]],
                         [0xa5,0x85,0xf,0x74,0,0])

    def test_signed_little_endian_scale_and_raw_body_axes(self):
        self.initialize(); self.lib.stub_data(-1234,2345,-3456,0x8f); self.lib.Mag_Service(10000,True)
        sample=self.sample(); status=self.status()
        self.assertEqual(list(status.raw),[-1234,2345,-3456])
        for actual,raw in zip(sample.raw,[-1234,2345,-3456]): self.assertAlmostEqual(actual,raw*.008,places=4)
        self.assertEqual(list(sample.field),list(sample.raw))
        self.assertEqual((sample.timestamp,sample.sequence,sample.flags),(10000,1,1))
        self.assertEqual(status.overruns,1)
        self.assertFalse(self.lib.Mag_PopSample(c.byref(sample)))

    def test_wrong_identity_never_writes_configuration(self):
        self.lib.stub_set(15,0x48); self.lib.Mag_Service(10000,True)
        self.assertEqual((self.status().state,self.status().error),(3,1))
        self.assertEqual(self.lib.stub_writes(),0)

    def test_readback_and_bus_errors_latch(self):
        self.lib.stub_ignore(0x24)
        for i in range(20): self.lib.Mag_Service(i*10,True)
        self.assertEqual((self.status().state,self.status().error,self.status().reg),(3,3,0x24))
        self.setUp(); self.lib.stub_error(1); self.lib.Mag_Service(0,True)
        self.assertEqual((self.status().state,self.status().error),(3,2))
        self.lib.stub_error(0); self.lib.Mag_Service(1000,True)
        self.assertEqual(self.status().state,3)

    def test_busy_and_not_ready_do_not_create_samples(self):
        self.lib.stub_busy(1)
        for i in range(20): self.lib.Mag_Service(i*10,True)
        self.assertEqual((self.status().state,self.status().error,self.lib.stub_writes()),(1,0,0))
        self.lib.stub_busy(0); self.initialize(); self.lib.Mag_Service(10000,True)
        self.assertEqual((self.status().samples,self.status().not_ready),(0,1))

    def test_missing_data_and_runtime_configuration_loss(self):
        self.initialize(); self.lib.Mag_Service(200000,True)
        self.assertEqual((self.status().state,self.status().error),(3,4))
        self.setUp(); self.initialize(); self.lib.stub_set(15,0)
        for i in range(1,202):
            self.lib.stub_data(1000,2000,3000,15); self.lib.Mag_Service(i*5000,True)
        self.assertEqual((self.status().state,self.status().error,self.status().reg),(3,3,15))

    def test_saturation_has_no_valid_flag_and_recovers_on_new_data(self):
        self.initialize(); self.lib.stub_data(32767,12,34,15); self.lib.Mag_Service(10000,True)
        sample=self.sample(); self.assertEqual(sample.flags,0)
        self.assertEqual((self.status().sample_valid,self.status().invalid),(0,1))
        self.lib.stub_data(1000,2000,3000,15); self.lib.Mag_Service(20000,True)
        self.assertEqual(self.sample().flags,1)

    def test_crc_validated_calibration_and_fail_closed_fallback(self):
        for index,valid in [(1,True),(2,False)]:
            with self.subTest(valid=valid):
                self.lib=self.libraries[index]; self.lib.stub_reset(); self.lib.Mag_Init(0); self.initialize()
                self.lib.stub_data(1000,2000,3000,15); self.lib.Mag_Service(10000,True)
                sample=self.sample(); status=self.status()
                for actual,expected in zip(sample.field,[14,14,10] if valid else [8,16,24]):
                    self.assertAlmostEqual(actual,expected,places=4)
                self.assertEqual((status.calibrated,status.calibration_error),(int(valid),int(not valid)))

    def test_actual_board_parameter_header_passes_native_crc_gate(self):
        self.lib=self.libraries[3]; self.lib.stub_reset(); self.lib.Mag_Init(0)
        self.assertEqual((self.status().calibrated,self.status().calibration_error),(1,0))


if __name__=='__main__': unittest.main()
