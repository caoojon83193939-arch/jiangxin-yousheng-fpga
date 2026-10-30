"""Check package hashes, exact final RTL identity and project/IP dependency closure."""
from pathlib import Path
import hashlib,json,sys,xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    files=json.loads((ROOT/'FILE_MANIFEST.json').read_text(encoding='utf-8'))['files']
    for name,row in files.items():
        p=(ROOT/name).resolve()
        assert p.is_relative_to(ROOT.resolve()) and p.is_file(),name
        assert p.stat().st_size==row['bytes'] and sha(p)==row['sha256'],name
    original=json.loads((ROOT/'verification/original_build_manifest.json').read_text(encoding='utf-8'))
    mapping=json.loads((ROOT/'verification/source_map.json').read_text(encoding='utf-8'))
    for name,row in mapping.items():
        assert sha(ROOT/name)==row['sha256'],name
        if row['original_build_input']:assert row['sha256']==original['sources_sha256'][row['original_path']],name
    refs=[]
    for node in ET.parse(ROOT/'media_player.al').findall('.//File'):
        p=(ROOT/node.attrib['Path']).resolve()
        assert p.is_relative_to(ROOT.resolve()) and p.is_file(),str(p)
        refs.append(p)
        if p.suffix=='.ipc':
            for child in ET.parse(p).findall('./GeneratedFiles/*'):
                if child.attrib.get('Enable')=='true':assert (p.parent/child.text).is_file(),child.text
    assert sha(ROOT/'bitstream/media_player.bit')==original['bit_sha256']
    assert len(list((ROOT/'tf_card').iterdir()))==13
    print('PASS',len(files),'file hashes;',len(refs),'project entries; all enabled IP files; original RTL/constraints/ROM and bitstream identity.')
if __name__=='__main__':main()
