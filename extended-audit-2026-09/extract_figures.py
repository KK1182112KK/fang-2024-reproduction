"""Extract original vector curve coordinates from the supplied Fang PDF.

No OCR, image tracing, fitting of controller parameters, or author-code access.
Calibration is explicit and specific to the inspected nine-page PDF. Coordinates
are publication graphics, NOT author raw data. Tick/coordinate quantization
limits precision, particularly when a large axis range magnifies rounding.
The source PDF is not redistributed; pass its local path to --pdf.
"""
from pathlib import Path
import argparse,json,hashlib
import numpy as np
import fitz

CAL={
 1:(4,[52,53],359.987,519.276,98.063,19.763),
 2:(4,[118,119],358.559,519.276,188.088,1.8527),
 3:(4,[181,182],355.702,519.276,320.194,19.2605/50000),
 4:(4,[244,245],356.416,519.276,444.698,16.8535/500000),
 5:(5,[52,53],90.999,250.288,98.063,19.763),
 6:(5,[118,119],89.571,250.288,188.089,1.8527),
}

def extract(pdf,out):
    doc=fitz.open(pdf)
    if len(doc)!=9:raise ValueError('Expected the supplied nine-page published PDF')
    out.mkdir(parents=True,exist_ok=True);metadata={}
    for fig,(page,paths,x0,x1,yzero,scale) in CAL.items():
        drawings=doc[page].get_drawings();curves=[]
        for index in paths:
            path=drawings[index]
            if len(path['items'])<1000:raise ValueError('PDF drawing structure differs from calibrated version')
            pts=[]
            for item in path['items']:
                if item[0]!='l':raise ValueError('Expected line segments')
                pts.extend([(item[1].x,item[1].y),(item[2].x,item[2].y)])
            a=np.array(pts);t=(a[:,0]-x0)/(x1-x0)*10.;y=(yzero-a[:,1])/scale
            order=np.argsort(t,kind='stable');t=t[order];y=y[order]
            unique=np.r_[True,np.diff(t)>1e-12];t=t[unique];y=y[unique]
            grid=np.linspace(0,10,1001);curve=np.interp(grid,t,y);curves.append(curve)
        np.savetxt(out/f'figure_{fig}_vector.csv',np.column_stack([grid,*curves]),delimiter=',',header='t,component_1,component_2',comments='')
        metadata[str(fig)]=dict(pdf_page=page+1,path_indices=paths,x0=x0,x1=x1,yzero=yzero,points_per_unit=scale,
                approximate_initial_values=[float(v[0]) for v in curves],approximate_final_values=[float(v[-1]) for v in curves])
    (out/'extraction_metadata.json').write_text(json.dumps(dict(pdf_sha256=hashlib.sha256(pdf.read_bytes()).hexdigest(),calibration=metadata,method=__doc__),indent=2)+'\n')

def compare(out):
    from audit import simulate
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    runs={'predictor':np.loadtxt('continuous_results/continuous_h0.0005.csv',delimiter=',',skiprows=1)}
    rows=[]
    for mode in ['uncompensated','delay_free']:
        t,x,u,p=simulate(dt=.001,mode=mode,order=1)
        runs[mode]=np.column_stack([t,x,u,p]);np.savetxt(out/(mode+'_euler.csv'),runs[mode],delimiter=',',header='t,x1,x2,u1,u2,P1,P2',comments='')
    for fig in range(1,7):
        mode='predictor' if fig<=2 else ('uncompensated' if fig<=4 else 'delay_free')
        source=np.loadtxt(out/f'figure_{fig}_vector.csv',delimiter=',',skiprows=1);run=runs[mode]
        cols=[1,2] if fig%2 else [3,4]
        y=np.column_stack([np.interp(source[:,0],run[:,0],run[:,c]) for c in cols])
        diff=y-source[:,1:3]
        rows.append(dict(figure=fig,scenario=mode,comparison='continuous-feedback h=.0005' if mode=='predictor' else 'Euler h=.001',
                    max_componentwise_graph_difference=np.max(abs(diff),axis=0).tolist(),
                    rms_graph_difference=np.sqrt(np.mean(diff*diff,axis=0)).tolist(),
                    simulated_initial_values=run[0,cols].tolist(),vector_graph_initial_values=source[0,1:3].tolist()))
        figob,ax=plt.subplots(figsize=(8,4.5))
        for ch in [0,1]:
            ax.plot(source[:,0],source[:,ch+1],label=f'Paper vector curve {ch+1}')
            ax.plot(source[:,0],y[:,ch],'--',label=f'Independent simulation {ch+1}')
        ax.set(xlabel='Time (s)',ylabel='state' if fig%2 else 'input',title=f'Fang Figure {fig}: published vector graphics versus simulation')
        ax.grid(True,alpha=.25);ax.legend(fontsize=8);figob.tight_layout();figob.savefig(out/f'figure_{fig}_comparison.png',dpi=160);plt.close(figob)
    (out/'comparison.json').write_text(json.dumps(rows,indent=2)+'\n');print(json.dumps(rows,indent=2))

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--pdf',type=Path,required=True);ap.add_argument('--out',type=Path,default=Path('figure_audit'));args=ap.parse_args()
    extract(args.pdf,args.out);compare(args.out)
