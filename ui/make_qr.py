#!/usr/bin/env python3
"""Draw the operator-console connect QR onto the robot's desktop.

The Jetson's address changes with whatever network it joins, so a QR printed
once goes stale the first time the robot moves rooms.  This regenerates it from
the address the machine is actually reachable on, so the picture on the desktop
is never a lie.

    python3 ui/make_qr.py                 # write it once and exit
    python3 ui/make_qr.py --watch         # rewrite whenever the address changes
    python3 ui/make_qr.py --print-url     # just say where the console lives

Run it at boot so the QR is correct before anyone reaches for a phone:

    [Unit]
    Description=X3Plus console connect QR
    After=network-online.target
    Wants=network-online.target

    [Service]
    ExecStart=/usr/bin/python3 /home/jetson/deploy_jetson2/ui/make_qr.py --watch
    Restart=always
    User=jetson

    [Install]
    WantedBy=graphical.target

Needs `pip3 install qrcode pillow`.
"""

import argparse
import socket
import sys
import time
from pathlib import Path

DEFAULT_PORT = 8080
# Re-checking every few seconds is enough: an address change means the robot
# just joined a network, and nobody scans the code in the same breath.
WATCH_PERIOD_S = 5.0

CAPTION = "X3Plus 操作台"
HINT = "掃描後直接進入，不需輸入 IP"


def lan_address(probe: str = "8.8.8.8") -> str:
    """The address this machine presents on its own network.

    ``hostname -I`` lists every interface -- docker bridges, ROS veths, the lot
    -- with no way to tell which one a phone can reach.  Opening a UDP socket
    toward an off-network address makes the kernel pick the interface it would
    actually route through, and reports it back.  No packet is ever sent.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((probe, 1))
        return s.getsockname()[0]
    except OSError:
        # No route anywhere -- offline, or only loopback is up.
        return ""
    finally:
        s.close()


def console_url(ip: str, port: int) -> str:
    # A freshly generated code may enter the phone console immediately; there
    # is no reason to ask the operator to confirm the same IP a second time.
    return "http://{ip}:{port}/?connect=1".format(ip=ip, port=port)


def load_font(size: int):
    from PIL import ImageFont
    for path in ("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
                 "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
                 "C:/Windows/Fonts/msjhbd.ttc",
                 "C:/Windows/Fonts/msjh.ttc",
                 "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
                 "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            continue
    return ImageFont.load_default()


def render(url: str, ip: str, out: Path, port: int = DEFAULT_PORT,
           caption: str = CAPTION, hint: str = HINT) -> None:
    try:
        import qrcode
        from PIL import Image, ImageDraw
    except ImportError:
        sys.exit("缺少相依套件。請先執行：pip3 install qrcode pillow")

    qr = qrcode.QRCode(box_size=10, border=2,
                       error_correction=qrcode.constants.ERROR_CORRECT_M)
    qr.add_data(url)
    qr.make(fit=True)
    code = qr.make_image(fill_color="#0F1B2A", back_color="white").convert("RGB")

    pad, footer = 28, 112
    card = Image.new("RGB", (code.width + pad * 2, code.height + pad + footer), "white")
    card.paste(code, (pad, pad))

    draw = ImageDraw.Draw(card)

    def centered(text: str, y: int, f, fill: str) -> None:
        # textbbox is the only width call present across the Pillow versions
        # that ship on Jetson images; textsize was removed in Pillow 10.
        try:
            box = draw.textbbox((0, 0), text, font=f)
            w = box[2] - box[0]
        except AttributeError:
            w = draw.textlength(text, font=f)
        draw.text(((card.width - w) / 2, y), text, font=f, fill=fill)

    top = pad + code.height + 4
    centered(caption, top, load_font(26), "#0F1B2A")
    centered("{}:{}".format(ip, port), top + 34, load_font(30), "#1256B8")
    centered(hint, top + 72, load_font(17), "#71849A")

    out.parent.mkdir(parents=True, exist_ok=True)
    card.save(out)


def desktop_dir() -> Path:
    for name in ("Desktop", "桌面"):
        p = Path.home() / name
        if p.is_dir():
            return p
    return Path.home()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT,
                    help="操作台服務的連接埠（預設 %(default)s）")
    ap.add_argument("--out", type=Path, default=None,
                    help="輸出檔路徑（預設為桌面上的 x3plus_connect.png）")
    ap.add_argument("--watch", action="store_true",
                    help="持續執行，IP 變了就重畫")
    ap.add_argument("--print-url", action="store_true",
                    help="只印出網址，不畫圖")
    args = ap.parse_args()

    out = args.out or desktop_dir() / "x3plus_connect.png"

    if args.print_url:
        ip = lan_address()
        if not ip:
            return print("找不到對外的 IP，請確認網路已連上。") or 1
        print(console_url(ip, args.port))
        return 0

    last = None
    while True:
        ip = lan_address()
        if not ip:
            # Booting alongside the network stack, or the Wi-Fi dropped.  Say so
            # once and keep looking rather than writing a QR for an address that
            # will not answer.
            if last != "":
                print("尚未取得 IP，等待網路…")
                last = ""
        elif ip != last:
            url = console_url(ip, args.port)
            render(url, ip, out, args.port)
            print("QR 已更新：{}\n  {}".format(out, url))
            last = ip

        if not args.watch:
            return 0 if last else 1
        time.sleep(WATCH_PERIOD_S)


if __name__ == "__main__":
    sys.exit(main())
