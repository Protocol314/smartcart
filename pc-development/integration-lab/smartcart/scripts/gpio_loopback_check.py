#!/usr/bin/env python3
"""모터 드라이버와 무관한 Pi GPIO5→GPIO6 연결만 확인한다.

GPIO5(물리 29)와 GPIO6(물리 31)은 시험 전에 다른 장치에서 분리해야 한다.
이 검사는 모터 구동 신호, 물리 정지 또는 주행 안전 검수가 아니다.
"""
import argparse
import time

OUTPUT_PIN = 5   # Pi 물리 29번
INPUT_PIN = 6    # Pi 물리 31번


def check(gpio, *, sleep=time.sleep):
    """두 핀만 초기화하고 관찰 후 출력 LOW 및 해당 핀 정리를 시도한다."""
    configured = []
    result = []
    try:
        gpio.setmode(gpio.BCM)
        gpio.setup(INPUT_PIN, gpio.IN, pull_up_down=gpio.PUD_DOWN)
        configured.append(INPUT_PIN)
        gpio.setup(OUTPUT_PIN, gpio.OUT, initial=gpio.LOW)
        configured.append(OUTPUT_PIN)
        for level in (gpio.LOW, gpio.HIGH, gpio.LOW):
            gpio.output(OUTPUT_PIN, level)
            sleep(.1)
            measured = gpio.input(INPUT_PIN)
            result.append((int(level), int(measured)))
        return result
    finally:
        try:
            if OUTPUT_PIN in configured:
                gpio.output(OUTPUT_PIN, gpio.LOW)
        finally:
            if configured:
                gpio.cleanup(configured)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pins-isolated', action='store_true',
                        help='5·6번에 다른 장치가 없고 두 핀이 점퍼로 연결됐음을 확인')
    args = parser.parse_args()
    if not args.pins_isolated:
        parser.error('5·6번 핀 분리와 점퍼 연결 확인이 필요합니다')
    import RPi.GPIO as gpio
    results = check(gpio)
    for sent, read in results:
        print(f'출력 {sent} → 입력 {read}')
    if any(sent != read for sent, read in results):
        raise SystemExit('루프백 불일치: 배선이나 핀 사용 상태를 확인하십시오')
    print('루프백 확인 완료 · 모터 출력 및 구동 검수 아님')


if __name__ == '__main__':
    main()
