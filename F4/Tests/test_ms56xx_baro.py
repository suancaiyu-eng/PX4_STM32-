"""Run the actual barometer C with a timed SPI device; no hardware is simulated
by test expectations alone. Datasheet examples cover both incompatible models.
"""
import ctypes as c
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
U32 = c.c_uint32
I32 = c.c_int32


class Sample(c.Structure):
    _fields_ = [(n, U32) for n in ('timestamp', 'sequence', 'flags', 'd1', 'd2')] + [
        (n, I32) for n in ('pressure', 'temperature', 'p5611', 'p5607')] + [
        ('altitude', c.c_float), ('filtered', c.c_float)]


class Status(c.Structure):
    _fields_ = [(n, U32) for n in ('state', 'error', 'model', 'samples', 'bus_errors',
        'busy_skips', 'timestamp', 'max_interval', 'reference_samples', 'reference_resets',
        'crc_expected', 'crc_actual', 'flags')] + [
        ('prom', c.c_uint16 * 8), ('reference', c.c_float)]


STUB = r'''
#include "ms56xx_baro.h"
#include <assert.h>
#include <string.h>
static uint16_t prom[8];
static uint32_t clock_us, started, d1, d2, ready_delay;
static unsigned conversion, reads, calls;
static int busy, fail;
void stub_reset(void) {
 uint16_t example[8]={0,40127,36924,23317,23282,33464,28312,0};
 memcpy(prom,example,sizeof(prom)); prom[7]=Baro_Crc4(prom);
 clock_us=started=conversion=reads=calls=busy=fail=0;
 d1=9085466; d2=8569150; ready_delay=0;
}
void stub_time(uint32_t value) { clock_us=value; }
void stub_busy(int value) { busy=value; }
void stub_fail(int value) { fail=value; }
void stub_delay(uint32_t value) { ready_delay=value; }
void stub_prom(unsigned index, unsigned value) { prom[index]=(uint16_t)value; }
void stub_raw(uint32_t p,uint32_t t) { d1=p; d2=t; }
unsigned stub_calls(void) { return calls; }
unsigned stub_reads(void) { return reads; }
uint32_t Baro_Port_NowUs(void) { return clock_us; }
int Baro_Port_Transfer(const uint8_t *tx,uint8_t *rx,uint16_t n) {
 unsigned value=0; ++calls;
 if(busy) return 0; if(fail) return -1;
 memset(rx,0,n);
 if(tx[0]==0x1e) { assert(n==1); started=clock_us; conversion=0; }
 else if(tx[0]>=0xa0 && tx[0]<=0xae) {
  assert(n==3 && (uint32_t)(clock_us-started)>=3000);
  value=prom[(tx[0]-0xa0)/2]; rx[1]=(uint8_t)(value>>8); rx[2]=(uint8_t)value;
 } else if(tx[0]==0x48 || tx[0]==0x58) {
  assert(n==1 && !conversion); clock_us+=ready_delay;
  started=clock_us; conversion=tx[0];
 } else {
  assert(tx[0]==0 && n==4 && conversion && (uint32_t)(clock_us-started)>=10000);
  value=conversion==0x48 ? d1 : d2; conversion=0; ++reads;
  rx[1]=(uint8_t)(value>>16); rx[2]=(uint8_t)(value>>8); rx[3]=(uint8_t)value;
 }
 return 1;
}
'''


class BaroTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        folder = Path(cls.directory.name)
        (folder / 'stub.c').write_text(STUB)
        cls.libs = {}
        for model in (0, 5611, 5607):
            path = folder / f'baro{model}.dll'
            result = subprocess.run([shutil.which('gcc'), '-std=c99', '-O2', '-Wall',
                '-Wextra', '-Werror', '-Wno-misleading-indentation', '-shared',
                f'-DBARO_SENSOR_MODEL={model}', '-I', str(ROOT / 'Core/Inc'),
                str(ROOT / 'Core/Src/ms56xx_baro.c'), str(folder / 'stub.c'),
                '-lm', '-o', str(path)], capture_output=True, text=True, timeout=30)
            if result.returncode: raise AssertionError(result.stdout + result.stderr)
            lib = c.CDLL(str(path))
            lib.Baro_Service.argtypes = [U32, c.c_bool]
            lib.Baro_GetLatest.argtypes = [U32, c.POINTER(Sample)]
            lib.Baro_GetLatest.restype = c.c_bool
            lib.Baro_GetStatus.argtypes = [c.POINTER(Status)]
            lib.Baro_PopSample.argtypes = [c.POINTER(Sample)]
            lib.Baro_PopSample.restype = c.c_bool
            lib.Baro_ZeroDisarmed.argtypes = [c.c_bool]
            lib.Baro_ZeroDisarmed.restype = c.c_bool
            lib.Baro_Compensate.argtypes = [U32, c.POINTER(c.c_uint16), U32, U32,
                c.POINTER(I32), c.POINTER(I32)]
            lib.Baro_Compensate.restype = c.c_bool
            lib.Baro_Crc4.argtypes = [c.POINTER(c.c_uint16)]
            lib.Baro_Crc4.restype = c.c_uint8
            cls.libs[model] = lib

    @classmethod
    def tearDownClass(cls):
        import _ctypes
        for lib in cls.libs.values(): _ctypes.FreeLibrary(lib._handle)
        cls.directory.cleanup()

    def start(self, model=5611, now=0):
        self.lib = self.libs[model]
        self.now = now
        self.lib.stub_reset()
        self.lib.stub_time(U32(now))
        self.lib.Baro_Init(U32(now))

    def setUp(self): self.start()

    def tick(self, count=1, imu=True):
        for _ in range(count):
            self.now = (self.now + 1000) & 0xffffffff
            self.lib.stub_time(U32(self.now))
            calls = self.lib.stub_calls()
            self.lib.Baro_Service(U32(self.now), imu)
            self.assertLessEqual(self.lib.stub_calls() - calls, 1)

    def status(self):
        out = Status(); self.lib.Baro_GetStatus(c.byref(out)); return out

    def sample(self):
        out = Sample(); valid = self.lib.Baro_GetLatest(U32(self.now), c.byref(out))
        return valid, out

    def test_datasheet_examples(self):
        for model, coeffs, d1, d2, pressure, temp in [
            (5611, [40127,36924,23317,23282,33464,28312],9085466,8569150,100009,2007),
            (5607, [46372,43981,29059,27842,31553,28165],6465444,8077636,110002,2000)]:
            prom = (c.c_uint16 * 8)(0,*coeffs,0); p=I32(); t=I32()
            self.assertTrue(self.lib.Baro_Compensate(model,prom,d1,d2,c.byref(p),c.byref(t)))
            self.assertEqual((p.value,t.value),(pressure,temp))

    def test_low_temperature_both_models(self):
        # Independent floating equations include <-15C and T2, missing in the
        # reference firmware. A 2Pa tolerance allows integer division stages.
        coeffs = [40127,36924,23317,23282,33464,28312]
        prom = (c.c_uint16 * 8)(0,*coeffs,0)
        for model in (5611,5607):
            for d2 in (8000000,7200000):
                dt=d2-coeffs[4]*256; temp=2000+dt*coeffs[5]/2**23
                sq=(temp-2000)**2; cold=(temp+1500)**2 if temp < -1500 else 0
                off=coeffs[1]*(2**16 if model==5611 else 2**17)+coeffs[3]*dt/(128 if model==5611 else 64)
                sens=coeffs[0]*(2**15 if model==5611 else 2**16)+coeffs[2]*dt/(256 if model==5611 else 128)
                off-=(2.5 if model==5611 else 61/16)*sq+(7 if model==5611 else 15)*cold
                sens-=(1.25 if model==5611 else 2)*sq+(5.5 if model==5611 else 8)*cold
                expected_p=(9085466*sens/2**21-off)/2**15
                expected_t=temp-dt**2/2**31
                p=I32(); t=I32()
                self.assertTrue(self.lib.Baro_Compensate(model,prom,9085466,d2,c.byref(p),c.byref(t)))
                self.assertAlmostEqual(p.value,expected_p,delta=3)
                self.assertAlmostEqual(t.value,expected_t,delta=2)

    def test_unknown_model_collects_both_without_height(self):
        self.start(0); self.tick(4000)
        valid,sample=self.sample()
        self.assertFalse(valid); self.assertEqual(sample.flags,1)
        self.assertEqual((sample.p5611,sample.p5607),(100009,200018))
        self.assertEqual(self.status().reference_samples,0)
        self.assertFalse(self.lib.Baro_ZeroDisarmed(True))

    def test_ms5607_acquisition_and_reference(self):
        self.start(5607)
        coeffs=[46372,43981,29059,27842,31553,28165]
        prom=(c.c_uint16*8)(0,*coeffs,0)
        crc=self.lib.Baro_Crc4(prom)
        for i,value in enumerate([0,*coeffs,crc]): self.lib.stub_prom(i,value)
        self.lib.stub_raw(6465444,8077636); self.tick(4000)
        valid,sample=self.sample()
        self.assertTrue(valid); self.assertEqual(sample.flags,7)
        self.assertEqual((sample.pressure,sample.temperature),(110002,2000))
        self.assertAlmostEqual(self.status().reference,110002,delta=1)

    def test_unstable_reference_restarts_and_range_failure(self):
        for _ in range(30):
            self.lib.stub_raw(9085466,8569150); self.tick(60)
            self.lib.stub_raw(9080000,8569150); self.tick(60)
        self.assertGreater(self.status().reference_resets,0)
        self.assertFalse(self.sample()[1].flags & 4)
        self.lib.stub_raw(9085466,8569150); self.tick(2500)
        self.assertTrue(self.sample()[1].flags & 4)
        self.lib.stub_raw(1,8569150); self.tick(30)
        self.assertEqual(self.status().error,5)
        self.assertEqual(self.sample()[1].flags,0)

    def test_known_crc_vector_and_invalid_arguments(self):
        # Fixed nontrivial PROM fixture; the stored CRC nibble is excluded.
        prom=(c.c_uint16*8)(0x3132,0x3334,0x3536,0x3738,0x3941,0x4243,0x4445,0x450b)
        self.assertEqual(self.lib.Baro_Crc4(prom),6)
        prom[7]=0x4506
        self.assertEqual(self.lib.Baro_Crc4(prom),6)
        p=I32(); t=I32()
        self.assertFalse(self.lib.Baro_Compensate(0,prom,1,1,c.byref(p),c.byref(t)))
        self.assertFalse(self.lib.Baro_Compensate(5611,prom,0,1,c.byref(p),c.byref(t)))
        self.assertFalse(self.lib.Baro_Compensate(5611,None,1,1,c.byref(p),c.byref(t)))

    def test_reference_height_sign_filter_and_zero(self):
        self.tick(4000)
        valid,sample=self.sample()
        self.assertTrue(valid); self.assertEqual(sample.flags,7)
        self.assertAlmostEqual(self.status().reference,100009,delta=1)
        self.assertAlmostEqual(sample.altitude,0,delta=0.01)
        self.assertFalse(self.lib.Baro_ZeroDisarmed(False))
        self.lib.stub_raw(9080000,8569150); self.tick(25)
        valid,sample=self.sample()
        self.assertTrue(valid); self.assertGreater(sample.altitude,5)
        self.assertGreater(sample.filtered,0); self.assertLess(sample.filtered,sample.altitude)
        self.assertTrue(self.lib.Baro_ZeroDisarmed(True))
        self.assertEqual(self.sample()[1].flags,3)
        self.tick(2500); self.assertEqual(self.sample()[1].flags,7)

    def test_spi_busy_retries_and_times_out(self):
        self.tick(100); seq=self.status().samples
        self.lib.stub_busy(1); self.tick(20)
        self.assertEqual(self.status().samples,seq)
        self.assertGreater(self.status().busy_skips,0)
        self.lib.stub_busy(0); self.tick(30)
        self.assertGreater(self.status().samples,seq)
        self.lib.stub_busy(1); self.tick(150)
        self.assertEqual((self.status().state,self.status().error),(3,6))
        self.assertEqual(self.sample()[1].flags,0)
        self.assertFalse(self.lib.Baro_PopSample(c.byref(Sample())))

    def test_prom_crc_and_blank(self):
        self.lib.stub_prom(1,40126); self.tick(30)
        self.assertEqual(self.status().error,3)
        for blank in (0,65535):
            self.start(); self.lib.stub_prom(1,blank); self.tick(30)
            self.assertEqual(self.status().error,2)

    def test_adc_bus_and_imu_faults(self):
        for raw in (0,0xffffff):
            self.start(); self.lib.stub_raw(raw,8569150); self.tick(100)
            self.assertEqual(self.status().error,4)
        self.start(); self.lib.stub_fail(1); self.tick()
        self.assertEqual(self.status().error,1)
        self.start(); self.tick(100); self.tick(1,False)
        self.assertEqual(self.status().error,7)
        self.assertEqual(self.sample()[1].flags,0)

    def test_latest_stale_without_service_and_rollover(self):
        self.start(now=0xffff0000); self.tick(4000)
        self.assertTrue(self.sample()[0]); self.assertEqual(self.sample()[1].flags,7)
        self.now=(self.now+100001)&0xffffffff
        self.lib.stub_time(U32(self.now))
        self.assertEqual(self.sample()[1].flags,0)
        self.assertFalse(self.lib.Baro_ZeroDisarmed(True))
        out=Sample()
        self.assertTrue(self.lib.Baro_PopSample(c.byref(out)))
        self.assertEqual(out.flags,0)

    def test_pending_single_consume_and_fault_latched(self):
        self.tick(100)
        self.assertTrue(self.lib.Baro_PopSample(c.byref(Sample())))
        self.assertFalse(self.lib.Baro_PopSample(c.byref(Sample())))
        self.lib.stub_fail(1); self.tick(20); self.lib.stub_fail(0); self.tick(500)
        self.assertEqual(self.status().state,3)

    def test_reset_and_conversion_deadline_uses_transaction_end(self):
        self.lib.stub_delay(400)
        self.tick(100)
        self.assertGreater(self.status().samples,1)
        self.assertEqual(self.status().error,0)


if __name__ == '__main__': unittest.main()
