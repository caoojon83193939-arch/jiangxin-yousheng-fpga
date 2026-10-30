"""Build the final standalone TD project; configure the installed TD root."""
from pathlib import Path
import argparse,os,sys,subprocess,runpy
ROOT=Path(__file__).resolve().parents[1]
def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--td-root',default=os.environ.get('JXYS_TD_ROOT','D:/Anlogic/TD_6.2.1_Engineer_6.2.168.116'))
    ap.add_argument('--end-step',default='bitgen',choices=['read_design','opt_rtl','opt_gate','opt_place','opt_route','bitgen'])
    args=ap.parse_args()
    td=Path(args.td_root).resolve()
    exe=td/'bin/td_commands_prompt.exe'
    default_flow=td/'doc/scripts/DefaultFlow.tcl'
    if not exe.is_file() or not default_flow.is_file():ap.error('TD installation missing; set --td-root')
    # Regenerate the project and make ROM files available in TD's run directory.
    runpy.run_path(str(ROOT/'gen_project.py'),run_name='__main__')
    build=ROOT/'build/cli';build.mkdir(parents=True,exist_ok=True)
    for name in ['spectrum_hann.hex','spectrum_twiddle.hex','info_font.hex','heritage_glyph.hex']:
        (build/name).write_bytes((ROOT/name).read_bytes())
    flow=(ROOT/'tools/flow_settings.tcl').read_text(encoding='utf-8')+'source {'+default_flow.as_posix()+'}\n'
    (build/'flow.tcl').write_text(flow,encoding='utf-8')
    # Remove old step markers only in the generated build directory.
    (build/('.'+args.end_step+'.end.f')).unlink(missing_ok=True)
    sys.path.insert(0,str(ROOT/'tools'))
    import td_build
    td_build.TD_ROOT=td;td_build.TD_CLI=exe;td_build.FLOW_TCL=Path('flow.tcl')
    sys.argv=['td_build.py',str(ROOT),'media_player','--build-dir',str(build),'--end-step',args.end_step]
    code=td_build.main()
    log=(build/'build.log').read_text(encoding='utf-8',errors='replace')
    bad=[s for s in log.splitlines() if 'ERROR:' in s or 'Error:' in s or 'FATAL' in s]
    if code or bad or not (build/('.'+args.end_step+'.end.f')).exists():
        raise RuntimeError('TD step failed: '+str(bad[:8]))
    if args.end_step=='bitgen':
        import re
        text=(build/'media_player_phy.ts').read_text()
        vals={n:float(re.search(n+r':\s*([-\d.]+)ns',text).group(1)) for n in ['SWNS','STNS','HWNS','HTNS']}
        if vals['SWNS']<0 or vals['HWNS']<0 or vals['STNS']!=0 or vals['HTNS']!=0:raise RuntimeError('Timing failed: '+str(vals))
        print('New build timing:',vals)
    print('PASS TD '+args.end_step)
if __name__=='__main__':main()
