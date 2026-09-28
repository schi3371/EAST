#!/usr/bin/env python3
"""Generate the dimensioned EAST A4 ArUco reference and colour-target sheet."""

from pathlib import Path
import io

import cv2
from PIL import Image
from reportlab.lib.colors import HexColor, black, white
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas
from reportlab.lib.utils import ImageReader

DICT_NAME = "DICT_4X4_50"
MARKER_SIZE_MM = 45.0
MARKERS = {
    0: (15.0, 20.0),
    1: (150.0, 20.0),
    2: (150.0, 232.0),
    3: (15.0, 232.0),
}


def marker_reader(marker_id: int, pixels: int = 720) -> ImageReader:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = cv2.aruco.generateImageMarker(dictionary, marker_id, pixels, borderBits=1)
    image = Image.fromarray(marker).convert("L")
    stream = io.BytesIO()
    image.save(stream, format="PNG")
    stream.seek(0)
    return ImageReader(stream)


def top_y(y_from_top_mm: float, height_mm: float = 0.0) -> float:
    return A4[1] - (y_from_top_mm + height_mm) * mm


def draw_scale_check(c: canvas.Canvas, y_mm: float) -> None:
    x = 55 * mm
    y = top_y(y_mm)
    c.setStrokeColor(black)
    c.setLineWidth(1)
    c.line(x, y, x + 100 * mm, y)
    for tick in range(0, 101, 10):
        height = 4 if tick % 50 else 7
        c.line(x + tick * mm, y - height * mm / 2, x + tick * mm, y + height * mm / 2)
    c.setFont("Helvetica", 8)
    c.drawCentredString(x + 50 * mm, y - 7 * mm, "100 mm print-scale check")


def draw_page_one(c: canvas.Canvas) -> None:
    c.setTitle("EAST ArUco reference board and colour targets")
    c.setFont("Helvetica-Bold", 15)
    c.drawCentredString(A4[0] / 2, top_y(8), "EAST CORNER-MOUNTED ARUCO MARKERS")
    c.setFont("Helvetica", 8.5)
    c.drawCentredString(A4[0] / 2, top_y(14), "A4 portrait | print at Actual Size / 100% | DICT_4X4_50 | cut out all four cards")

    cards = {0: (15, 25), 1: (115, 25), 3: (15, 145), 2: (115, 145)}
    marker_size = 70.0
    for marker_id, (x_mm, y_mm) in cards.items():
        c.setStrokeColor(HexColor("#888888")); c.setDash(3, 2)
        c.rect((x_mm - 7) * mm, top_y(y_mm - 7, marker_size + 20), (marker_size + 14) * mm, (marker_size + 20) * mm)
        c.setDash(); c.setStrokeColor(black)
        c.drawImage(
            marker_reader(marker_id),
            x_mm * mm,
            top_y(y_mm, marker_size),
            marker_size * mm,
            marker_size * mm,
            mask="auto",
        )
        c.setFont("Helvetica-Bold", 8)
        c.drawCentredString(
            (x_mm + marker_size / 2) * mm,
            top_y(y_mm + marker_size + 5),
            f"ID {marker_id} | 70 mm black square",
        )
    c.setFont("Helvetica", 7.5)
    c.drawCentredString(A4[0] / 2, top_y(281), "Mount on rigid matte cards. Keep at least 10 mm of clean white border around each black square.")
    draw_scale_check(c, 289)
    c.showPage()


def draw_target(c: canvas.Canvas, cx_mm: float, cy_mm: float, diameter_mm: float, colour, label: str) -> None:
    radius = diameter_mm / 2
    c.setFillColor(colour)
    c.setStrokeColor(black)
    c.setLineWidth(0.5)
    c.circle(cx_mm * mm, top_y(cy_mm), radius * mm, fill=1, stroke=1)
    c.setFillColor(black)
    c.circle(cx_mm * mm, top_y(cy_mm), 1.0 * mm, fill=1, stroke=0)
    c.setFont("Helvetica-Bold", 6.5)
    c.drawCentredString(cx_mm * mm, top_y(cy_mm + radius + 5), label)


def draw_page_two(c: canvas.Canvas) -> None:
    magenta = HexColor("#EC008C")
    yellow = HexColor("#FFF200")
    c.setFont("Helvetica-Bold", 15)
    c.setFillColor(black)
    c.drawCentredString(A4[0] / 2, top_y(10), "EAST COLOUR TARGETS - CUT-OUT PAGE")
    c.setFont("Helvetica", 8.5)
    c.drawCentredString(A4[0] / 2, top_y(17), "Print at Actual Size / 100% on matte white paper; mount targets flat on stiff card or matte vinyl")

    draw_target(c, 50, 55, 40, magenta, "MAGENTA - moving")
    draw_target(c, 160, 55, 40, yellow, "YELLOW - moving")
    draw_target(c, 50, 115, 40, magenta, "MAGENTA - spare")
    draw_target(c, 160, 115, 40, yellow, "YELLOW - spare")
    draw_target(c, 45, 177, 50, magenta, "MAGENTA 50 mm")
    draw_target(c, 165, 177, 50, yellow, "YELLOW 50 mm")

    c.setFont("Helvetica-Bold", 10)
    c.drawString(20 * mm, top_y(218), "Placement")
    c.setFont("Helvetica", 8.5)
    instructions = [
        "1. Put both colours on the same rigid moving component whose rotation represents AFO angle.",
        "2. Put yellow nearer the joint and magenta farther away; keep their centres at least 80 mm apart.",
        "3. The line from yellow to magenta defines the measured orientation. Neither target must be on the fixed frame.",
        "4. Keep both targets flat, fully visible, and near the same depth plane as the four ArUco markers.",
    ]
    y = 227
    for line in instructions:
        c.drawString(20 * mm, top_y(y), line)
        y += 6

    c.setFont("Helvetica-Bold", 9)
    c.drawString(20 * mm, top_y(256), "Colour check")
    c.setFillColor(magenta)
    c.rect(44 * mm, top_y(264, 10), 35 * mm, 10 * mm, fill=1, stroke=1)
    c.setFillColor(yellow)
    c.rect(83 * mm, top_y(264, 10), 35 * mm, 10 * mm, fill=1, stroke=1)
    c.setFillColor(black)
    c.setFont("Helvetica", 8)
    c.drawString(122 * mm, top_y(271), "Use these patches when checking the mask preview.")
    draw_scale_check(c, 284)
    c.showPage()


def draw_page_three(c: canvas.Canvas) -> None:
    c.setFont("Helvetica-Bold", 15)
    c.drawCentredString(A4[0] / 2, top_y(10), "EAST CAMERA CALIBRATION INSTALLATION RECORD")
    c.setFont("Helvetica", 9)
    c.drawCentredString(A4[0] / 2, top_y(18), "Install around the visible movement area on stationary frame members")

    x0, x1, y0, y1 = 30, 180, 45, 155
    c.setStrokeColor(HexColor("#666666")); c.setLineWidth(1.2)
    c.rect(x0 * mm, top_y(y0, y1-y0), (x1-x0)*mm, (y1-y0)*mm)
    positions = [(0,x0,y0),(1,x1,y0),(2,x1,y1),(3,x0,y1)]
    for marker_id,x,y in positions:
        c.setFillColor(white); c.setStrokeColor(black)
        c.rect((x-10)*mm, top_y(y-10,20), 20*mm,20*mm, fill=1,stroke=1)
        c.setFillColor(black); c.setFont("Helvetica-Bold",10)
        c.drawCentredString(x*mm, top_y(y+2), f"ID {marker_id}")
    c.setFillColor(HexColor("#FFF200")); c.circle(95*mm,top_y(105),7*mm,fill=1,stroke=1)
    c.setFillColor(HexColor("#EC008C")); c.circle(125*mm,top_y(105),7*mm,fill=1,stroke=1)
    c.setFillColor(black); c.setFont("Helvetica",8)
    c.drawCentredString(110*mm,top_y(118),"Yellow and magenta move together")

    c.setFont("Helvetica-Bold",10); c.drawString(20*mm,top_y(178),"Installation measurements")
    c.setFont("Helvetica",9)
    fields=[
        "Date / setup ID: __________________________________________",
        "ID 0 centre to ID 1 centre: __________________________ mm",
        "ID 0 centre to ID 3 centre: __________________________ mm",
        "ID 1 centre to ID 2 centre: __________________________ mm",
        "ID 3 centre to ID 2 centre: __________________________ mm",
        "Diagonal ID 0 to ID 2: _______________________________ mm",
        "Diagonal ID 1 to ID 3: _______________________________ mm",
        "Yellow-to-magenta centre spacing: ____________________ mm",
    ]
    y=188
    for line in fields:
        c.drawString(25*mm,top_y(y),line); y+=8
    c.setFont("Helvetica-Bold",9); c.drawString(20*mm,top_y(257),"Acceptance before recording")
    c.setFont("Helvetica",8)
    checks=[
        "[ ] All four IDs visible throughout full ROM   [ ] Markers fixed to stationary frame",
        "[ ] Camera perpendicular and tripod locked     [ ] Targets on one rigid moving member",
        "[ ] Initial 2 s recorded at machine zero       [ ] Static -10, 0, +10 degree check completed",
    ]
    y=266
    for line in checks:
        c.drawString(25*mm,top_y(y),line); y+=7
    c.showPage()


def main() -> None:
    output = Path(__file__).resolve().parent / "EAST_ArUco_A4_Marker_Kit.pdf"
    output.parent.mkdir(parents=True, exist_ok=True)
    c = canvas.Canvas(str(output), pagesize=A4, pageCompression=1)
    draw_page_one(c)
    draw_page_two(c)
    draw_page_three(c)
    c.save()
    print(output)


if __name__ == "__main__":
    main()
