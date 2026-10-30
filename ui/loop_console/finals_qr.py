#!/usr/bin/env python3
"""Robot desktop icon: double-click to show the QR for the finals console.

The finals console runs on the PC, not the Jetson, and the PC's address changes
with the network, so a fixed picture would go stale. The PC is found from who
is ssh-ed into the robot (the console's pose stream stays connected), and an
address only gets a QR after it answers as a console.

    python3 ui/loop_console/finals_qr.py --install     # put the icon on the desktop
    python3 ui/loop_console/finals_qr.py               # what the icon runs
    python3 ui/loop_console/finals_qr.py --print-url
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
from urllib.request import urlopen

sys.path.append(str(Path(__file__).resolve().parents[1]))
import make_qr  # noqa: E402

PORT = 8090
CAPTION = '決賽操作台'
HINT = '手機連同一個 Wi-Fi，掃描後進入'
ICON_NAME = '決賽操作台 QR'
LAST_ADDRESS = Path.home() / '.cache' / 'x3plus_finals_console'
QR_IMAGE = Path.home() / '.cache' / 'x3plus_finals_qr.png'
ICON_IMAGE = Path.home() / '.local' / 'share' / 'icons' / 'x3plus_finals_qr.png'


def ssh_peers(ss_output):
    """The IPv4 addresses in `ss -tn` output, in order, without repeats."""
    peers = []
    for line in ss_output.splitlines()[1:]:
        fields = line.split()
        if not fields:
            continue
        host = fields[-1].rsplit(':', 1)[0].strip('[]').replace('::ffff:', '')
        if ':' not in host and host not in peers:
            peers.append(host)
    return peers


def answers(ip, port):
    try:
        with urlopen('http://{}:{}/api/state'.format(ip, port), timeout=2) as reply:
            state = json.loads(reply.read().decode())
    except (OSError, ValueError):
        return False
    return isinstance(state, dict) and 'stage' in state and 'link' in state


def pick(candidates, port):
    for ip in candidates:
        if answers(ip, port):
            return ip
    return None


def find_console(port):
    listing = subprocess.run(['ss', '-tn', 'state', 'established', '( sport = :22 )'],
                             stdout=subprocess.PIPE, universal_newlines=True).stdout
    candidates = ssh_peers(listing)
    if LAST_ADDRESS.exists():      # the pose stream may be between reconnects
        candidates.append(LAST_ADDRESS.read_text().strip())
    found = pick(candidates, port)
    if found:
        LAST_ADDRESS.parent.mkdir(parents=True, exist_ok=True)
        LAST_ADDRESS.write_text(found)
    return found


def draw_icon(out):
    import qrcode
    from PIL import Image, ImageDraw
    qr = qrcode.QRCode(box_size=8, border=1, error_correction=qrcode.constants.ERROR_CORRECT_H)
    qr.add_data(CAPTION)
    qr.make(fit=True)
    code = qr.make_image(fill_color='#0F1B2A', back_color='white').convert('RGB')
    icon = code.resize((256, 256), Image.NEAREST)
    draw = ImageDraw.Draw(icon)
    draw.rectangle((48, 92, 208, 164), fill='#C8102E', outline='white', width=5)
    font = make_qr.load_font(54)
    box = draw.textbbox((0, 0), '決賽', font=font)
    draw.text((128 - (box[2]+box[0])/2, 128 - (box[3]+box[1])/2), '決賽', font=font, fill='white')
    out.parent.mkdir(parents=True, exist_ok=True)
    icon.save(str(out))


def install():
    draw_icon(ICON_IMAGE)
    shortcut = make_qr.desktop_dir() / '{}.desktop'.format(ICON_NAME)
    shortcut.write_text('\n'.join((
        '[Desktop Entry]', 'Version=1.0', 'Type=Application', 'Name={}'.format(ICON_NAME),
        'Comment=顯示決賽操作台的手機連線 QR Code',
        'Exec=/usr/bin/python3 "{}"'.format(Path(__file__).resolve()),
        'Icon={}'.format(ICON_IMAGE), 'Terminal=false', 'Categories=Utility;', '')))
    shortcut.chmod(0o755)
    # Nautilus asks before the first launch unless the file is marked trusted.
    env = dict(os.environ)
    env.setdefault('DBUS_SESSION_BUS_ADDRESS', 'unix:path=/run/user/{}/bus'.format(os.getuid()))
    subprocess.run(['gio', 'set', str(shortcut), 'metadata::trusted', 'yes'], env=env, check=True)
    print(shortcut)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', type=int, default=PORT)
    parser.add_argument('--install', action='store_true')
    parser.add_argument('--print-url', action='store_true')
    args = parser.parse_args()
    if args.install:
        return install()
    ip = find_console(args.port)
    if ip is None:
        message = '找不到決賽操作台。\n請先在電腦上啟動操作台，並確認電腦跟機器人在同一個網路。'
        if args.print_url:
            sys.exit(message)
        subprocess.call(['zenity', '--error', '--no-wrap', '--title', ICON_NAME, '--text', message])
        return 1
    url = 'http://{}:{}/'.format(ip, args.port)
    if args.print_url:
        return print(url)
    make_qr.render(url, ip, QR_IMAGE, args.port, caption=CAPTION, hint=HINT)
    subprocess.Popen(['eog', str(QR_IMAGE)])


if __name__ == '__main__':
    sys.exit(main())
