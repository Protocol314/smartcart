"""Reuse a trusted certificate when possible; never bypass certificate verification."""
import ipaddress
import json
import os
from pathlib import Path
import secrets
import shutil
import subprocess
import sys

NETWORKS=tuple(ipaddress.IPv4Network(n) for n in ('10.0.0.0/8','172.16.0.0/12','192.168.0.0/16'))

def private_ip(value):
    address=ipaddress.IPv4Address(value)
    if not any(address in n for n in NETWORKS):
        raise ValueError('RFC1918 private IPv4 required')
    return str(address)

def choose_address():
    data=json.loads(subprocess.check_output(['ip','-j','-4','addr'],text=True))
    for name in ('wlan0','eth0'):
        for interface in data:
            if interface.get('ifname')!=name:
                continue
            for entry in interface.get('addr_info',[]):
                if entry.get('scope')=='global':
                    try:return private_ip(entry['local'])
                    except ValueError:pass
    raise ValueError('No wlan0/eth0 private IPv4 found; pass an address explicitly')

def checked(args):
    subprocess.run(['openssl',*map(str,args)],check=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE)

def verified(folder,address):
    try:
        if not all((folder/n).is_file() for n in ('server.crt','server.key','ca.crt')):
            return False
        checked(['verify','-CAfile',folder/'ca.crt','-verify_ip',address,folder/'server.crt'])
        checked(['pkey','-in',folder/'server.key','-check','-noout'])
        # Test that the certificate and key really match, without exposing either.
        import ssl
        ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER).load_cert_chain(folder/'server.crt',folder/'server.key')
        return True
    except (OSError,subprocess.CalledProcessError,ValueError):
        return False


def reusable_real_ca(folder):
    try:
        if not all((folder/n).is_file() for n in ('ca.crt','ca.key')):
            return False
        subject = subprocess.check_output([
            'openssl','x509','-in',str(folder/'ca.crt'),
            '-noout','-subject','-nameopt','RFC2253'
        ], stderr=subprocess.DEVNULL, text=True).strip()
        if subject != 'subject=CN=SmartCart Real v1 Local CA':
            return False
        extensions = subprocess.check_output([
            'openssl','x509','-in',str(folder/'ca.crt'),'-noout','-text'
        ], stderr=subprocess.DEVNULL, text=True)
        if 'Name Constraints' in extensions:
            return False
        checked(['verify','-check_ss_sig','-CAfile',
                 folder/'ca.crt',folder/'ca.crt'])
        public_cert = subprocess.check_output([
            'openssl','x509','-in',str(folder/'ca.crt'),'-noout','-pubkey'
        ], stderr=subprocess.DEVNULL)
        public_key = subprocess.check_output([
            'openssl','pkey','-in',str(folder/'ca.key'),'-pubout'
        ], stderr=subprocess.DEVNULL)
        return public_cert.replace(b'\r\n',b'\n') == public_key.replace(b'\r\n',b'\n')
    except (OSError, subprocess.CalledProcessError, ValueError):
        return False

def prepare(root,address):
    address=private_ip(address)
    base=root/'certs'/'real-v1'
    if base.is_symlink() or not base.resolve().is_relative_to(root.resolve()):
        raise ValueError('Certificate base must not be a symlink')
    existing=[root/'smartcart-virtual-lan-cert',root/'smartcart-radar-https-check-PyDCFCjn/cert']
    if os.environ.get('SMARTCART_CERT_DIR'):
        existing.insert(0,Path(os.environ['SMARTCART_CERT_DIR']).expanduser())
    if base.is_dir():
        existing=list(sorted(base.glob('ip-*'),reverse=True))+existing
    for folder in existing:
        if verified(folder,address):
            return folder,False
    base.mkdir(parents=True,exist_ok=True,mode=0o700)
    folder=base/('ip-'+address+'-'+secrets.token_hex(4))
    folder.mkdir(mode=0o700)
    ca_source=next((d for d in existing if reusable_real_ca(d)),None)
    old_umask=os.umask(0o077)
    try:
        new_ca=ca_source is None
        if ca_source is None:
            ca_source=folder
            checked(['req','-x509','-newkey','rsa:2048','-nodes','-days','3650',
                     '-keyout',folder/'ca.key','-out',folder/'ca.crt',
                     '-subj','/CN=SmartCart Real v1 Local CA',
                     '-addext','basicConstraints=critical,CA:TRUE',
                     '-addext','keyUsage=critical,keyCertSign,cRLSign'])
        else:
            shutil.copy2(ca_source/'ca.crt',folder/'ca.crt')
        checked(['req','-new','-newkey','rsa:2048','-nodes','-keyout',folder/'server.key',
                 '-out',folder/'server.csr','-subj','/CN=SmartCart Real v1'])
        extension=folder/'server.ext'
        extension.write_text('subjectAltName=IP:'+address+'\nbasicConstraints=critical,CA:FALSE\nkeyUsage=critical,digitalSignature,keyEncipherment\nextendedKeyUsage=serverAuth\n')
        checked(['x509','-req','-in',folder/'server.csr','-CA',ca_source/'ca.crt',
                 '-CAkey',ca_source/'ca.key','-set_serial',str(secrets.randbits(127)+1),
                 '-days','365','-sha256','-extfile',extension,'-out',folder/'server.crt'])
        if not verified(folder,address):
            raise ValueError('Generated certificate verification failed')
        return folder,new_ca
    finally:
        os.umask(old_umask)

if __name__=='__main__':
    root=Path.home()/'smartcart'
    address=private_ip(sys.argv[1]) if len(sys.argv)>1 else choose_address()
    folder,new=prepare(root,address)
    print('IP:',address,'\nCertificate directory:',folder)
    if new:print('NEW CA: install and trust this ca.crt on the browser device; do not bypass TLS warnings.')
