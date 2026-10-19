"""Fit a full symmetric 3D hard/soft-iron correction from FW8 raw magnetic data.

Requires numpy. Never accepts a stationary/planar or poorly fitted data set.
The 50uT target radius sets normalization, not a measurement of local Earth field.
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import zlib
import numpy as np


def read_capture(path):
    rows=[]
    for line in Path(path).read_text(encoding='utf-8-sig').splitlines():
        if not line.startswith('mpu6000:'):
            continue
        parts=line[8:].split(',')
        if len(parts)!=18 or parts[17]!='1':
            continue
        try:
            value=[float(v) for v in parts[11:14]]
        except ValueError:
            continue
        if all(np.isfinite(value)) and max(abs(v) for v in value)<260:
            rows.append(value)
    return np.asarray(rows,dtype=float)


def ellipsoid(points):
    origin=points.mean(axis=0)
    scale=np.std(points,axis=0).mean()
    if scale<1:
        raise ValueError('Magnetic motion is insufficient')
    v=(points-origin)/scale
    design=np.column_stack([v[:,0]**2,v[:,1]**2,v[:,2]**2,
                            2*v[:,0]*v[:,1],2*v[:,0]*v[:,2],2*v[:,1]*v[:,2],v])
    condition=float(np.linalg.cond(design))
    if condition>100:
        raise ValueError('Ill-conditioned orientation coverage')
    weights=np.ones(len(points))
    for _ in range(5):
        root=np.sqrt(weights)
        p=np.linalg.lstsq(design*root[:,None],root,rcond=None)[0]
        residual=design@p-1
        sigma=max(1e-5,1.4826*np.median(abs(residual-np.median(residual))))
        weights=np.minimum(1,2.5*sigma/np.maximum(abs(residual),1e-12))
    a=np.array([[p[0],p[3],p[4]],[p[3],p[1],p[5]],[p[4],p[5],p[2]]])
    center=-0.5*np.linalg.solve(a,p[6:9])
    factor=1+center@a@center
    shape=a/factor/scale**2
    eigen,rotation=np.linalg.eigh(shape)
    if eigen.min()<=0 or eigen.max()/eigen.min()>25:
        raise ValueError('Nonphysical ellipsoid')
    matrix=50*(rotation*np.sqrt(eigen))@rotation.T
    bias=origin+scale*center
    if np.max(abs(bias))>200 or np.linalg.eigvalsh(matrix).min()<0.2 or np.linalg.eigvalsh(matrix).max()>5:
        raise ValueError('Calibration outside supported bounds')
    return bias,matrix,condition


def fit(points):
    if points.ndim!=2 or points.shape[1]!=3 or len(points)<500 or not np.isfinite(points).all():
        raise ValueError('Need at least 500 finite 3D magnetic samples')
    if np.ptp(points,axis=0).min()<20:
        raise ValueError('Rotate around all axes; each magnetic span must exceed 20uT')
    # Deterministic chronological holdout, not a claim of independent field/temperature testing.
    held=np.arange(len(points))%5==0
    bias,matrix,condition=ellipsoid(points[~held])
    transformed=(points-bias)@matrix.T
    radii=np.linalg.norm(transformed,axis=1)
    directions=transformed/np.maximum(radii[:,None],1e-12)
    octants=np.unique((directions>0).astype(int)@np.array([1,2,4]),return_counts=True)
    occupied=int(sum(octants[1]>=max(5,len(points)*0.005)))
    if occupied<6 or np.any(directions.min(axis=0)>-0.65) or np.any(directions.max(axis=0)<0.65):
        raise ValueError('Insufficient 3D directional coverage; need six octants and both ends of every axis')
    residual=abs(radii-50)/50
    held_rms=float(np.sqrt(np.mean((radii[held]/50-1)**2)))
    p95=float(np.percentile(residual[held],95))
    if held_rms>0.03 or p95>0.06:
        raise ValueError(f'Calibration residual too large: holdout RMS={held_rms:.4f}, P95={p95:.4f}')
    # Refit all observations only after the held-out model passed.
    bias,matrix,condition=ellipsoid(points)
    radius=np.linalg.norm((points-bias)@matrix.T,axis=1)
    report={'records':len(points),'bias_uT':bias.tolist(),'matrix':matrix.tolist(),
            'reference_uT':50.0,'holdout_records':int(held.sum()),'holdout_rms_relative':held_rms,
            'holdout_p95_relative':p95,'all_rms_relative':float(np.sqrt(np.mean((radius/50-1)**2))),
            'design_condition':condition,'octants':dict(zip(map(int,octants[0]),map(int,octants[1]))),
            'direction_min':directions.min(axis=0).tolist(),'direction_max':directions.max(axis=0).tolist(),
            'note':'50uT is a chosen normalization radius; heading accuracy still needs motion/environment checks'}
    return bias,matrix,report


def header(bias,matrix):
    # Decimal strings must round back to the same float32 values used for CRC.
    values=np.asarray([*bias,*matrix.reshape(-1),50.0],dtype=np.float32)
    crc=zlib.crc32(struct.pack('<13f',*values))
    def f(v):
        text=format(float(v),'.9g')
        if '.' not in text and 'e' not in text: text+='.0'
        return text+'f'
    bias_text='{'+','.join(f(v) for v in values[:3])+'}'
    matrix_text='{'+','.join('{'+','.join(f(v) for v in values[3+i*3:6+i*3])+'}' for i in range(3))+'}'
    return ('#ifndef MAG_CALIBRATION_PARAMS_H\n#define MAG_CALIBRATION_PARAMS_H\n'
            '/* Board-specific LSM303D full-ellipsoid calibration; see Logs/fw8_mag_calibration_fit.json. */\n'
            '#define MAG_CALIBRATION_VALID 1U\n'
            f'#define MAG_CALIBRATION_CRC32 0x{crc:08X}U\n'
            '#define MAG_CALIBRATION_REFERENCE_UT 50.0f\n'
            f'#define MAG_CALIBRATION_BIAS {bias_text}\n#define MAG_CALIBRATION_MATRIX {matrix_text}\n#endif\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--report',required=True,type=Path)
    args=parser.parse_args()
    if len({args.input.resolve(),args.output.resolve(),args.report.resolve()})!=3:
        parser.error('Capture, generated header and report must use different paths')
    bias,matrix,report=fit(read_capture(args.input))
    report['source_sha256']=hashlib.sha256(args.input.read_bytes()).hexdigest()
    args.output.write_text(header(bias,matrix),encoding='utf-8')
    args.report.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
