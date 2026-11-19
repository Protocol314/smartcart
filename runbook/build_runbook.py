#!/usr/bin/env python3
"""운영 런북 Markdown을 외부 의존성 없는 단일 HTML로 만들고 동기화를 확인한다."""
from __future__ import annotations

import argparse
import hashlib
import html
import os
from pathlib import Path
import re
import tempfile
import unittest
from urllib.parse import unquote, urlsplit


DEFAULT_SOURCE = Path(__file__).resolve().parent / "Smart-Cart Raspberry Pi 4 기본 운영 런북.md"
MAX_SOURCE_BYTES = 4 * 1024 * 1024
LANGUAGES = {"bash": "Bash", "sh": "Bash", "powershell": "PowerShell", "python": "Python",
             "json": "JSON", "text": "구성 / 출력", "sshconfig": "SSH 설정", "console": "터미널 출력"}
HEADING = re.compile(r"^(#{1,4})\s+(.+?)(?:\s+#+)?\s*$")
FENCE = re.compile(r"^\s{0,3}(`{3,}|~{3,})([^\s]*)\s*$")
LIST = re.compile(r"^( *)([-+*]|\d+[.)])\s+(.*)$")


def safe_link(value):
    """HTTP(S), 문서 내부 앵커와 상대 문서 경로만 링크로 만든다."""
    value = html.unescape(value.strip())
    if value.startswith("<") and value.endswith(">"):
        value = value[1:-1]
    decoded = unquote(value)
    if (not value or len(value) > 4096 or any(ord(char) < 32 for char in decoded)
            or "\\" in decoded or decoded.startswith("//")):
        return None
    try:
        parsed = urlsplit(decoded)
        if parsed.scheme:
            if parsed.scheme.lower() not in ("http", "https") or not parsed.hostname or parsed.username or parsed.password:
                return None
        elif parsed.netloc or decoded.startswith("/") or ":" in decoded.split("/", 1)[0]:
            return None
    except ValueError:
        return None
    return value


def inline(value, depth=0):
    """허용한 문법만 HTML로 만들고 나머지는 모두 문자 그대로 이스케이프한다."""
    if depth > 8:
        return html.escape(value, quote=False)
    parts, index = [], 0
    while index < len(value):
        if value[index] == "\\" and index + 1 < len(value) and value[index + 1] in r"`*[]()|\\":
            parts.append(html.escape(value[index + 1], quote=False))
            index += 2
            continue
        if value[index] == "`":
            marker = re.match(r"`+", value[index:]).group()
            end = value.find(marker, index + len(marker))
            if end != -1:
                text = value[index + len(marker):end].replace("\n", " ")
                parts.append("<code>" + html.escape(text, quote=False) + "</code>")
                index = end + len(marker)
                continue
        if value.startswith("**", index):
            end = value.find("**", index + 2)
            if end != -1:
                parts.append("<strong>" + inline(value[index + 2:end], depth + 1) + "</strong>")
                index = end + 2
                continue
        if value[index] == "[":
            label_end = value.find("](", index + 1)
            if label_end != -1:
                cursor, nesting, quoted = label_end + 2, 1, False
                while cursor < len(value):
                    char = value[cursor]
                    if char == "<" and nesting == 1:
                        quoted = True
                    elif char == ">":
                        quoted = False
                    elif not quoted:
                        if char == "(":
                            nesting += 1
                        elif char == ")":
                            nesting -= 1
                            if nesting == 0:
                                break
                    cursor += 1
                if nesting == 0:
                    label = inline(value[index + 1:label_end], depth + 1)
                    destination = safe_link(value[label_end + 2:cursor])
                    parts.append(("<a href=\"" + html.escape(destination, quote=True) + "\">" + label + "</a>")
                                 if destination is not None else label)
                    index = cursor + 1
                    continue
        parts.append(html.escape(value[index], quote=False))
        index += 1
    return "".join(parts)


def plain_text(value):
    value = re.sub(r"\[([^\]]+)\]\((?:<[^>]+>|[^)]+)\)", r"\1", value)
    return value.replace("`", "").replace("**", "")


def heading_slug(value):
    value = plain_text(value).strip().lower()
    # GitHub의 일반적인 제목 앵커 규칙: 구두점 제외, 공백은 하이픈으로 유지한다.
    value = re.sub(r"[^\w\s-]", "", value, flags=re.UNICODE)
    return re.sub(r"\s", "-", value)


def table_cells(line):
    """인라인 코드 안의 파이프와 이스케이프한 파이프는 열 구분자로 쓰지 않는다."""
    value = line.strip()
    if value.startswith("|"):
        value = value[1:]
    if value.endswith("|") and not value.endswith("\\|"):
        value = value[:-1]
    cells, current, ticks, index = [], [], 0, 0
    while index < len(value):
        char = value[index]
        if char == "\\" and index + 1 < len(value):
            current.extend((char, value[index + 1]))
            index += 2
            continue
        if char == "`":
            marker = re.match(r"`+", value[index:]).group()
            ticks = 0 if ticks == len(marker) else len(marker)
            current.append(marker)
            index += len(marker)
            continue
        if char == "|" and not ticks:
            cells.append("".join(current).strip())
            current = []
        else:
            current.append(char)
        index += 1
    cells.append("".join(current).strip())
    return cells


def table_start(lines, index):
    if index + 1 >= len(lines) or "|" not in lines[index]:
        return False
    separator = table_cells(lines[index + 1])
    return bool(separator) and all(re.fullmatch(r":?-{3,}:?", cell.replace(" ", "")) for cell in separator)


class Renderer:
    def __init__(self):
        self.headings = []
        self.slug_counts = {}
        self.code_count = 0

    def slug(self, title):
        base = heading_slug(title) or "section"
        count = self.slug_counts.get(base, 0)
        self.slug_counts[base] = count + 1
        return base if count == 0 else base + "-" + str(count)

    def block_start(self, lines, index):
        value = lines[index]
        return bool(HEADING.match(value) or FENCE.match(value) or LIST.match(value)
                    or value.startswith(">") or re.fullmatch(r"\s*(?:---+|\*\*\*+|___+)\s*", value)
                    or table_start(lines, index))

    def list_block(self, lines, index):
        first = LIST.match(lines[index])
        base = len(first.group(1))
        ordered = first.group(2)[0].isdigit()
        tag = "ol" if ordered else "ul"
        start = ' start="' + re.match(r"\d+", first.group(2)).group() + '"' if ordered and not first.group(2).startswith("1.") else ""
        items = []
        while index < len(lines):
            match = LIST.match(lines[index])
            if not match or len(match.group(1)) != base or match.group(2)[0].isdigit() != ordered:
                break
            text = match.group(3)
            checkbox = re.match(r"^\[([ xX])\]\s+(.*)$", text)
            body = [checkbox.group(2) if checkbox else text]
            index += 1
            while index < len(lines):
                candidate = lines[index]
                nested = LIST.match(candidate)
                if nested and len(nested.group(1)) <= base:
                    break
                if not candidate.strip():
                    if index + 1 < len(lines) and (len(lines[index + 1]) - len(lines[index + 1].lstrip(" "))) > base:
                        body.append("")
                        index += 1
                        continue
                    index += 1
                    break
                indent = len(candidate) - len(candidate.lstrip(" "))
                if indent <= base:
                    if self.block_start(lines, index):
                        break
                    # 개행한 목록 문장은 한 항목에 계속 이어 쓴다.
                    body.append(candidate)
                else:
                    body.append(candidate[min(indent, base + 2):])
                index += 1
            rendered = self.blocks(body)
            if checkbox:
                checked = " checked" if checkbox.group(1).lower() == "x" else ""
                rendered = '<input type="checkbox" disabled aria-label="체크 항목"' + checked + ">" + rendered
            items.append('<li class="check-item">' + rendered + "</li>" if checkbox else "<li>" + rendered + "</li>")
        return "<" + tag + start + ">\n" + "\n".join(items) + "\n</" + tag + ">", index

    def blocks(self, lines):
        output, index = [], 0
        while index < len(lines):
            line = lines[index]
            if not line.strip():
                index += 1
                continue
            fence = FENCE.match(line)
            if fence:
                marker, language = fence.groups()
                content = []
                index += 1
                closing = re.compile(r"^\s{0,3}" + re.escape(marker[0]) + "{" + str(len(marker)) + r",}\s*$")
                while index < len(lines) and not closing.match(lines[index]):
                    content.append(lines[index])
                    index += 1
                if index >= len(lines):
                    raise ValueError("닫히지 않은 코드 블록이 있습니다")
                index += 1
                self.code_count += 1
                language = language.lower()
                label = LANGUAGES.get(language, "코드")
                css = language if re.fullmatch(r"[a-z0-9_-]{0,32}", language) else "text"
                identifier = "code-" + str(self.code_count)
                code = "\n".join(content) + ("\n" if content else "")
                output.append('<div class="code-block"><div class="code-toolbar"><span>' + label
                              + '</span><button type="button" class="copy-btn" data-code="' + identifier
                              + '" aria-label="' + label + ' 코드 복사">복사</button></div><pre tabindex="0"><code id="'
                              + identifier + '" class="language-' + css + '">' + html.escape(code, quote=False)
                              + "</code></pre></div>")
                continue
            heading = HEADING.match(line)
            if heading:
                level, title = len(heading.group(1)), heading.group(2)
                identifier = self.slug(title)
                self.headings.append((level, title, identifier))
                output.append("<h" + str(level) + ' id="' + html.escape(identifier, quote=True) + '">'
                              + inline(title) + "</h" + str(level) + ">")
                index += 1
                continue
            if re.fullmatch(r"\s*(?:---+|\*\*\*+|___+)\s*", line):
                output.append("<hr>")
                index += 1
                continue
            if line.startswith(">"):
                quote = []
                while index < len(lines) and lines[index].startswith(">"):
                    quote.append(re.sub(r"^> ?", "", lines[index], count=1))
                    index += 1
                output.append("<blockquote>" + self.blocks(quote) + "</blockquote>")
                continue
            if table_start(lines, index):
                headers = table_cells(line)
                if len(headers) != len(table_cells(lines[index + 1])):
                    raise ValueError("표의 제목과 구분선 열 수가 다릅니다")
                rows = []
                index += 2
                while index < len(lines) and lines[index].strip() and "|" in lines[index]:
                    cells = table_cells(lines[index])
                    if len(cells) != len(headers):
                        raise ValueError("표의 행마다 같은 열 수가 필요합니다: " + lines[index][:80])
                    rows.append("<tr>" + "".join("<td>" + inline(cell) + "</td>" for cell in cells) + "</tr>")
                    index += 1
                output.append('<div class="table-wrap" tabindex="0" role="region" aria-label="가로로 이동할 수 있는 표"><table><thead><tr>'
                              + "".join('<th scope="col">' + inline(cell) + "</th>" for cell in headers)
                              + "</tr></thead><tbody>" + "\n".join(rows) + "</tbody></table></div>")
                continue
            if LIST.match(line):
                rendered, index = self.list_block(lines, index)
                output.append(rendered)
                continue
            paragraph = [line.strip()]
            index += 1
            while index < len(lines) and lines[index].strip() and not self.block_start(lines, index):
                paragraph.append(lines[index].strip())
                index += 1
            output.append("<p>" + inline(" ".join(paragraph)) + "</p>")
        return "\n".join(output)


STYLE = r"""
:root{color-scheme:light;--ink:#17283b;--muted:#52677c;--accent:#087d79;--line:#d9e3e9;--soft:#f1f6f8;--navy:#102b42;--code:#122b3d}
*{box-sizing:border-box}html{scroll-behavior:smooth;scroll-padding-top:24px}body{margin:0;background:#f7fafb;color:var(--ink);font:16px/1.8 -apple-system,BlinkMacSystemFont,"Segoe UI","Noto Sans KR",Arial,sans-serif;word-break:keep-all;overflow-wrap:anywhere}a{color:#066b78;text-underline-offset:3px}a:hover{color:#004856}button,summary{font:inherit}button{cursor:pointer}a:focus-visible,button:focus-visible,summary:focus-visible,pre:focus-visible,.table-wrap:focus-visible{outline:3px solid #ea9d22;outline-offset:4px}.skip{position:absolute;left:16px;top:-80px;background:white;padding:8px 18px;z-index:10}.skip:focus{top:8px}.masthead{background:var(--navy);color:#fff;border-bottom:4px solid var(--accent)}.masthead-inner{max-width:1520px;margin:auto;padding:21px 40px;display:flex;justify-content:space-between;gap:20px;align-items:center}.brand{font-size:15px;font-weight:750;letter-spacing:.11em}.masthead-note{font-size:12px;letter-spacing:.04em;color:#c8d9e5}.layout{max-width:1520px;display:grid;grid-template-columns:260px minmax(0,1fr);gap:45px;margin:0 auto;padding:40px}.sidebar{min-width:0}.toc-shell{position:sticky;top:24px;border:1px solid var(--line);border-radius:12px;background:#fff;overflow:hidden}.toc-shell summary{padding:14px 18px;font-size:13px;font-weight:750;letter-spacing:.06em;color:var(--navy);cursor:pointer}.toc-shell nav{max-height:calc(100vh - 125px);overflow:auto;padding:0 13px 16px}.toc-shell a{display:block;text-decoration:none;color:var(--muted);font-size:12.5px;line-height:1.55;padding:7px 9px;border-left:2px solid transparent}.toc-shell a.level-3{padding-left:21px;font-size:11.5px}.toc-shell a:hover,.toc-shell a.active{color:#046c68;background:#e9f5f3;border-left-color:var(--accent)}main{min-width:0;background:#fff;border:1px solid var(--line);border-radius:15px;padding:35px 42px 55px;box-shadow:0 10px 30px #16314d05}h1,h2,h3,h4{color:#132c41;line-height:1.45;overflow-wrap:anywhere}h1{font-size:clamp(27px,2.6vw,38px);letter-spacing:-.045em;margin:0 0 24px;padding-bottom:24px;border-bottom:2px solid var(--navy)}h2{font-size:23px;margin:58px 0 20px;padding-top:24px;border-top:1px solid var(--line);letter-spacing:-.02em}h3{font-size:18px;margin:35px 0 13px}h4{font-size:16px;margin:25px 0 10px}p{margin:11px 0 16px}blockquote{background:#eaf5f3;border-left:4px solid var(--accent);border-radius:0 9px 9px 0;padding:13px 19px;margin:22px 0;font-size:14px}blockquote p{margin:3px 0}ul,ol{padding-left:24px;margin:12px 0 19px}li{padding-left:3px;margin:7px 0}li p{margin:3px 0}li>ul,li>ol{margin:8px 0}.check-item{list-style:none;position:relative}.check-item>input{position:absolute;left:-23px;top:8px;accent-color:var(--accent)}.table-wrap{max-width:100%;overflow-x:auto;border:1px solid var(--line);border-radius:9px;margin:19px 0 25px}table{width:100%;border-collapse:collapse;font-size:13px;line-height:1.7}th,td{padding:12px 14px;border-bottom:1px solid var(--line);text-align:left;vertical-align:top}th{background:#edf3f6;color:#1a3c53;font-weight:700;white-space:nowrap}td+td,th+th{border-left:1px solid #e4ebef}tbody tr:last-child td{border-bottom:0}tbody tr:nth-child(even){background:#fbfcfd}code{font: .88em/1.7 Consolas,"Cascadia Code","D2Coding",monospace;word-break:normal;background:#edf3f7;color:#174f69;padding:.13em .35em;border-radius:4px}td code{overflow-wrap:anywhere;white-space:normal}.code-block{margin:19px 0 25px;border:1px solid #203d50;border-radius:9px;overflow:hidden;background:var(--code)}.code-toolbar{display:flex;align-items:center;justify-content:space-between;gap:15px;padding:8px 13px;background:#19384c;color:#c1d5e0;font-size:11px;letter-spacing:.05em;border-bottom:1px solid #294b5c}.copy-btn{border:1px solid #496778;border-radius:5px;color:#eaf5f7;background:#23475b;padding:3px 11px;font-size:11px;min-width:52px}.copy-btn:hover{background:#2e5a70}pre{margin:0;padding:18px 19px;overflow:auto;tab-size:4;color:#ecf5fa;font-size:12.8px;line-height:1.75}pre code{display:block;background:none;color:inherit;padding:0;border-radius:0;white-space:pre;overflow-wrap:normal;font-size:inherit;line-height:inherit}.language-text{color:#b9e1db}hr{border:0;border-top:1px solid var(--line);margin:34px 0}.page-footer{max-width:1520px;margin:0 auto;padding:0 40px 35px;color:var(--muted);font-size:12px;display:flex;justify-content:space-between;gap:15px}.page-footer a{color:var(--muted)}.sr-only{position:absolute;width:1px;height:1px;padding:0;overflow:hidden;clip:rect(0,0,0,0);white-space:nowrap;border:0}@media(min-width:1600px){.layout{grid-template-columns:280px minmax(0,1fr);gap:55px}}@media(max-width:1050px){.layout{grid-template-columns:220px minmax(0,1fr);padding:28px;gap:25px}main{padding:28px}body{font-size:15px}.masthead-inner{padding:18px 28px}}@media(max-width:820px){.layout{display:block;padding:18px 16px}.sidebar{margin-bottom:18px}.toc-shell{position:static}.toc-shell nav{max-height:42vh}.masthead-inner{padding:15px 20px}.masthead-note{display:none}main{padding:24px 20px;border-radius:10px}h1{font-size:26px}h2{font-size:21px;margin-top:43px;padding-top:21px}h3{font-size:18px}pre{padding:15px;font-size:11.8px}.page-footer{padding:8px 20px 28px;display:block}.table-wrap{margin-left:0;margin-right:0}th,td{padding:10px 11px}table{min-width:510px;font-size:12px}blockquote{padding:11px 14px;font-size:13px}}@media(prefers-reduced-motion:reduce){html{scroll-behavior:auto}}@media print{@page{margin:16mm}body{background:#fff;font-size:10pt;line-height:1.6}.masthead,.sidebar,.copy-btn,.page-footer,.skip{display:none}.layout{display:block;padding:0;max-width:none}main{padding:0;border:0;box-shadow:none;border-radius:0}h1{font-size:24pt}h2{font-size:16pt;margin-top:26pt;break-after:avoid}h3,h4{break-after:avoid}.code-block{break-inside:avoid;background:#f4f6f8;border-color:#b7c4ce}.code-toolbar{background:#e8edf1;color:#102b42;border-color:#c4cdd5}pre{white-space:pre-wrap;font-size:8pt;color:#17283b;overflow:visible}pre code{white-space:pre-wrap;overflow-wrap:anywhere;color:#17283b}.table-wrap{overflow:visible;border-color:#bfcbd4}table{min-width:0;font-size:8.5pt}thead{display:table-header-group}tr{break-inside:avoid}blockquote{break-inside:avoid}a{color:inherit;text-decoration:underline}h1,h2,h3,h4{scroll-margin-top:0}}
"""

SCRIPT = r"""
(() => {
  'use strict';
  const status = document.getElementById('copy-status');
  const fallback = (value) => {
    const field = document.createElement('textarea');
    field.value = value;
    field.setAttribute('readonly', '');
    field.style.position = 'fixed';
    field.style.left = '-9999px';
    document.body.appendChild(field);
    field.select();
    let copied = false;
    try { copied = document.execCommand('copy'); } finally { field.remove(); }
    if (!copied) throw new Error('복사할 수 없습니다');
  };
  document.querySelectorAll('.copy-btn').forEach((button) => {
    button.addEventListener('click', async () => {
      const code = document.getElementById(button.dataset.code);
      if (!code) return;
      button.disabled = true;
      try {
        if (navigator.clipboard && window.isSecureContext) {
          try { await navigator.clipboard.writeText(code.textContent); }
          catch (_) { fallback(code.textContent); }
        } else { fallback(code.textContent); }
        button.textContent = '복사됨';
        status.textContent = '코드를 복사했습니다.';
      } catch (_) {
        button.textContent = '직접 선택';
        status.textContent = '코드를 선택해 복사하십시오.';
      } finally {
        button.disabled = false;
        button.focus({preventScroll: true});
        window.setTimeout(() => { button.textContent = '복사'; }, 1800);
      }
    });
  });
  const contents = document.querySelector('.toc-shell');
  if (window.matchMedia('(max-width: 820px)').matches) contents.removeAttribute('open');
  if ('IntersectionObserver' in window) {
    const links = [...document.querySelectorAll('.toc-shell a')];
    const observer = new IntersectionObserver((entries) => {
      entries.forEach((entry) => {
        if (!entry.isIntersecting) return;
        links.forEach((link) => link.classList.toggle('active', link.hash === '#' + encodeURIComponent(entry.target.id)
          || decodeURIComponent(link.hash.slice(1)) === entry.target.id));
      });
    }, {rootMargin: '-8% 0px -70% 0px'});
    document.querySelectorAll('main h2, main h3').forEach((heading) => observer.observe(heading));
  }
})();
"""


def render_document(markdown, source_digest=""):
    renderer = Renderer()
    body = renderer.blocks(markdown.replace("\r\n", "\n").replace("\r", "\n").split("\n"))
    title = next((plain_text(title) for level, title, _ in renderer.headings if level == 1), "스마트카트 운영 런북")
    minimum = 2 if any(level == 2 for level, _, _ in renderer.headings) else 1
    navigation = "\n".join('<a class="level-' + str(level) + '" href="#' + html.escape(identifier, quote=True)
                           + '">' + inline(title) + "</a>" for level, title, identifier in renderer.headings
                           if minimum <= level <= 3)
    return '<!doctype html>\n<html lang="ko">\n<head>\n<meta charset="utf-8">\n' \
        '<meta name="viewport" content="width=device-width, initial-scale=1">\n' \
        '<meta name="referrer" content="no-referrer">\n<meta name="source-sha256" content="' \
        + html.escape(source_digest, quote=True) + '">\n<title>' + html.escape(title, quote=False) + '</title>\n<style>' \
        + STYLE + '</style>\n</head>\n<body>\n<a class="skip" href="#main">본문으로 이동</a>\n' \
        '<header class="masthead" id="top"><div class="masthead-inner"><div class="brand">SMART CART · 운영 런북</div>' \
        '<div class="masthead-note">연결 · 설치 · 실행 · 점검</div></div></header>\n' \
        '<div class="layout"><aside class="sidebar"><details class="toc-shell" open><summary>문서 목차</summary>' \
        '<nav aria-label="문서 목차">' + navigation + '</nav></details></aside><main id="main">\n' + body \
        + '\n</main></div><footer class="page-footer"><span>Markdown 원본과 같은 내용을 담은 독립 HTML · 외부 자산 없음</span>' \
        '<a href="#top">맨 위로</a></footer><span class="sr-only" id="copy-status" aria-live="polite"></span>\n<script>' \
        + SCRIPT + '</script>\n</body>\n</html>\n'


class RendererTests(unittest.TestCase):
    def test_code_is_escaped_and_exact_copy_text_is_preserved(self):
        value = Renderer().blocks(['```bash', "printf '%s\\n' '<script>&\"'", '```'])
        self.assertIn("&lt;script&gt;&amp;\"", value)
        self.assertNotIn("<script>", value)
        self.assertIn("printf '%s\\n'", value)
        self.assertIn('data-code="code-1"', value)

    def test_raw_html_and_inline_code_are_never_executed(self):
        value = inline('<img src=x onerror=alert(1)> **중요** `<b>`')
        self.assertNotIn("<img", value)
        self.assertIn("&lt;img", value)
        self.assertIn("<strong>중요</strong>", value)
        self.assertIn("<code>&lt;b&gt;</code>", value)

    def test_links_reject_executable_network_and_credential_destinations(self):
        for destination in ('javascript:alert(1)', 'javascript&#58;alert(1)', 'data:text/html,bad', '//host/path',
                            'file:///secret', 'https://user:password@example.org', 'https://host/\npath', r'\\server\share'):
            with self.subTest(destination=destination):
                self.assertIsNone(safe_link(destination))
        self.assertEqual(safe_link('<../docs/설치 안내.md>'), '../docs/설치 안내.md')
        self.assertEqual(safe_link('https://example.org/docs'), 'https://example.org/docs')
        self.assertNotIn('href=', inline('[실행](javascript:alert(1))'))

    def test_table_keeps_inline_pipes_and_escapes_cells(self):
        renderer = Renderer()
        value = renderer.blocks(['| 명령 | 설명 |', '|---|---|', '| `a | b` | <script> |'])
        self.assertEqual(value.count('<td>'), 2)
        self.assertIn('<code>a | b</code>', value)
        self.assertIn('&lt;script&gt;', value)
        with self.assertRaises(ValueError):
            renderer.blocks(['| a | b |', '|---|---|', '| one |'])

    def test_heading_ids_and_toc_links_are_deterministic_and_unique(self):
        value = render_document('# 제목\n\n## 1. 연결 확인\n\n### USB\n\n### USB\n')
        self.assertIn('id="1-연결-확인"', value)
        self.assertIn('href="#1-연결-확인"', value)
        self.assertIn('id="usb-1"', value)
        self.assertEqual(value, render_document('# 제목\n\n## 1. 연결 확인\n\n### USB\n\n### USB\n'))

    def test_lists_nested_checkboxes_and_blockquotes(self):
        value = Renderer().blocks(['- [ ] 확인', '  - 중첩', '- 다음', '', '> **주의**', '> 연결 확인'])
        self.assertEqual(value.count('<ul>'), 2)
        self.assertIn('type="checkbox" disabled', value)
        self.assertIn('<blockquote>', value)
        self.assertIn('<strong>주의</strong>', value)

    def test_code_fences_must_close_and_html_fence_remains_text(self):
        with self.assertRaises(ValueError):
            Renderer().blocks(['```bash', 'echo x'])
        value = Renderer().blocks(['```html', '<script>x</script>', '```'])
        self.assertIn('&lt;script&gt;x&lt;/script&gt;', value)

    def test_page_has_offline_copy_and_accessibility_without_external_assets(self):
        value = render_document('# 제목\n\n## 연결\n\n```bash\necho ok\n```\n')
        self.assertIn('navigator.clipboard.writeText(code.textContent)', value)
        self.assertIn('aria-live="polite"', value)
        self.assertIn('@media(max-width:820px)', value)
        self.assertIn('@media print', value)
        self.assertNotIn('innerHTML', value)
        self.assertNotIn('<script src=', value)
        self.assertNotIn('<link ', value)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=DEFAULT_SOURCE, help='런북 Markdown 원본')
    parser.add_argument('--output', type=Path, help='출력 HTML. 기본은 원본과 같은 이름')
    parser.add_argument('--check', action='store_true', help='파일을 쓰지 않고 Markdown/HTML 동기화 확인')
    parser.add_argument('--self-test', action='store_true', help='이스케이프·링크·표·목차·복사 자기검사')
    args = parser.parse_args(argv)
    if args.self_test:
        result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(RendererTests))
        if not result.wasSuccessful():
            return 1
        if not args.check:
            return 0
    try:
        source = args.source.resolve(strict=True)
        output = (args.output or source.with_suffix('.html')).resolve()
        if not source.is_file() or source.stat().st_size > MAX_SOURCE_BYTES:
            raise ValueError('원본은 4 MiB 이하의 일반 Markdown 파일이어야 합니다')
        if output == source or output.suffix.lower() != '.html' or not output.parent.is_dir():
            raise ValueError('기존 디렉터리의 별도 HTML 출력 경로가 필요합니다')
        raw = source.read_bytes()
        if len(raw) > MAX_SOURCE_BYTES:
            raise ValueError('원본 크기 상한을 넘었습니다')
        rendered = render_document(raw.decode('utf-8-sig'), hashlib.sha256(raw).hexdigest()).encode('utf-8')
        if args.check:
            if not output.is_file() or output.read_bytes() != rendered:
                print('HTML이 Markdown 원본과 다릅니다. 생성기를 인수 없이 실행하십시오.')
                return 1
            print('Markdown·HTML 내용과 생성 결과가 일치합니다.')
            return 0
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(prefix='.runbook-', suffix='.tmp', dir=output.parent, delete=False) as stream:
                temporary = Path(stream.name)
                stream.write(rendered)
            os.replace(temporary, output)
        finally:
            if temporary is not None and temporary.exists():
                temporary.unlink()
        print('HTML 생성 완료: ' + str(output))
        return 0
    except (OSError, UnicodeError, ValueError, RecursionError) as error:
        print('런북 생성 오류: ' + str(error))
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
