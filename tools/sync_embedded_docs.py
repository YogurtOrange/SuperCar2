"""Refresh the offline wiring page's Markdown downloads; preserve embedded SVGs."""
import argparse
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PAGE = ROOT / 'docs' / 'wiring' / '共享串口接线与CubeMX配置总览.html'
PATTERN = re.compile(r'(<script id="embedded-sources" type="application/json">)(.*?)(</script>)', re.S)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='verify without writing')
    args = parser.parse_args()
    page = PAGE.read_text(encoding='utf-8')
    match = PATTERN.search(page)
    if not match:
        raise SystemExit('embedded-sources was not found')
    sources = json.loads(match.group(2))
    # New top-level project guides should also be downloadable from the offline page.
    for source in sorted((ROOT / 'docs').glob('*.md')):
        sources.setdefault(source.name, None)
    changed = []
    for name in sources:
        if name.endswith('.md'):
            source = ROOT / 'docs' / name
            content = source.read_text(encoding='utf-8')
            if sources[name] != content:
                changed.append(name)
                sources[name] = content
    if args.check:
        if changed:
            raise SystemExit('Outdated embedded docs: ' + ', '.join(changed))
        print('PASS embedded Markdown matches project docs')
        return
    if changed:
        # Escape '<' so embedded Markdown cannot terminate the JSON script tag.
        payload = json.dumps(sources, ensure_ascii=False).replace('<', r'\u003c')
        page = page[:match.start(2)] + payload + page[match.end(2):]
        PAGE.write_text(page, encoding='utf-8')
    print('Synced', len(changed), 'Markdown documents; preserved',
          sum(name.endswith('.svg') for name in sources), 'SVGs')


if __name__ == '__main__':
    main()
