#!/usr/bin/env python3
"""
Test lõi NLU mà KHÔNG cần cài ROS, không cần micro.

Chạy bộ test tự động:   python3 scripts/test_nlu.py
Chat thử tay:           python3 scripts/test_nlu.py -i
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
from airport_robot_voice.nlu import NluEngine   # noqa: E402

CFG = os.path.join(os.path.dirname(HERE), 'config')
engine = NluEngine(os.path.join(CFG, 'intents.json'),
                   os.path.join(CFG, 'places.json'))

TESTS = [
    # --- greeting / thanks / goodbye / capability ---
    ('xin chào robot', 'greeting', None, None),
    ('chào bạn', 'greeting', None, None),
    ('ê robot', 'greeting', None, None),
    ('cảm ơn bạn nhiều', 'thanks', None, None),
    ('cám ơn nhé', 'thanks', None, None),
    ('tạm biệt robot', 'goodbye', None, None),
    ('thôi tôi đi đây', 'goodbye', None, None),
    ('bạn giúp được gì cho tôi', 'capability', 'help', None),
    ('robot biết làm những gì vậy', 'capability', 'help', None),
    ('cách sử dụng robot', 'capability', 'help', None),

    # --- gate location (chuyển sang navigate theo spec 5.1 & 5.2) ---
    ('em ơi cổng a3 ở đâu', ('guide_me', 'ask_gate_location'), 'navigate', 'gate_a3'),
    ('cổng A3 ở đâu', ('guide_me', 'ask_gate_location'), 'navigate', 'gate_a3'),
    ('cổng a ba ở đâu', ('guide_me', 'ask_gate_location'), 'navigate', 'gate_a3'),
    ('dẫn tôi đến cổng a3', 'guide_me', 'navigate', 'gate_a3'),
    ('cho tôi tới cổng bê hai', 'guide_me', 'navigate', 'gate_b2'),
    ('cổng b2 chỗ nào', ('guide_me', 'ask_gate_location'), 'navigate', 'gate_b2'),
    ('cổng số 15 ở đâu ạ', 'ask_gate_location', 'navigate', 'gate_15'),
    ('cổng số mười lăm ở đâu', 'ask_gate_location', 'navigate', 'gate_15'),
    ('gate a1 ở chỗ nào', ('guide_me', 'ask_gate_location'), 'navigate', 'gate_a1'),
    ('dẫn tôi đến cổng a tư', 'guide_me', 'navigate', 'gate_a4'),

    # --- mã không tồn tại / word boundary ---
    ('dẫn tôi đến cổng a99', 'place_not_found', None, None),
    ('cổng b99 ở đâu', 'place_not_found', None, None),
    ('dẫn tôi đến cổng a123', 'place_not_found', None, None),
    ('dẫn tôi đến cổng a10', 'place_not_found', None, None),

    # --- checkin ---
    ('quầy check in ở đâu vậy', 'ask_checkin', 'navigate', 'checkin'),
    ('tôi làm thủ tục lên máy bay ở đâu', 'ask_checkin', 'navigate', 'checkin'),
    ('quay lam thu tuc o dau', 'ask_checkin', 'navigate', 'checkin'),
    ('cho tôi tới quầy check-in', 'guide_me', 'navigate', 'checkin'),

    # --- toilet (nhiều cách nói + category) ---
    ('cho tôi hỏi nhà vệ sinh ở đâu ạ', 'ask_toilet', 'navigate', None),
    ('toilet chỗ nào', 'ask_toilet', 'navigate', None),
    ('wc ở đâu vậy bạn', 'ask_toilet', 'navigate', None),
    ('tôi muốn đi vệ sinh quá', ('ask_toilet', 'guide_me'), 'navigate', None),
    ('nha ve sinh o dau', 'ask_toilet', 'navigate', None),
    ('dẫn tôi đến nhà vệ sinh', 'guide_me', 'navigate', None),
    ('dẫn tôi đến nhà vệ sinh sảnh A', 'guide_me', 'navigate', 'toilet_1'),

    # --- food / baggage / info desk / atm / security ---
    ('tôi đói quá', 'ask_food', 'navigate', 'food_court'),
    ('có quán cà phê nào gần đây không', 'ask_food', 'navigate', 'food_court'),
    ('băng chuyền hành lý chỗ nào', 'ask_baggage', 'navigate', 'baggage'),
    ('quầy thông tin ở đâu vậy bạn', 'ask_info_desk', 'navigate', 'info_desk'),
    ('cây atm ở đâu', 'ask_atm', 'navigate', 'atm'),
    ('cửa an ninh ở đâu ạ', 'ask_security', 'navigate', 'security'),

    # --- pause / resume / cancel / go_home ---
    ('dừng lại', 'pause_robot', 'pause', None),
    ('chờ tôi một chút', 'pause_robot', 'pause', None),
    ('tạm dừng đi bạn', 'pause_robot', 'pause', None),
    ('đi tiếp nào', 'resume_robot', 'resume', None),
    ('tiếp tục đi', 'resume_robot', 'resume', None),
    ('không cần nữa', 'cancel_robot', 'cancel', None),
    ('hủy chuyến đi', 'cancel_robot', 'cancel', None),
    ('quay về trạm sạc đi', 'go_home', 'navigate', 'home'),

    # --- phủ định ---
    ('đừng dẫn tôi đến cổng A1', 'negated_nav', None, None),
    ('không dừng lại nhé', 'negated_pause', None, None),

    # --- đa đích (ambiguous) & thiếu đích ---
    ('dẫn tôi đến cổng A1 hay A2', 'ambiguous_target', None, None),
    ('dẫn tôi đến', 'missing_destination', None, None),

    # --- offline services: wifi, charging, hours, status, time ---
    ('wifi sân bay tên gì', 'ask_wifi', None, None),
    ('sạc điện thoại ở đâu', 'ask_charging', None, None),
    ('sân bay mở cửa mấy giờ', 'ask_hours', None, None),
    ('bạn đang dẫn tôi đi đâu', 'ask_status', 'status', None),
    ('bây giờ là mấy giờ rồi', 'ask_time', None, None),

    # --- ngoài phạm vi ---
    ('hôm nay thời tiết thế nào', 'unknown', None, None),
    ('bạn có người yêu chưa', 'unknown', None, None),
    ('abcxyz lảm nhảm', 'unknown', None, None),
]


def run():
    ok = 0
    total = len(TESTS)
    for text, exp_intent, exp_action, exp_target in TESTS:
        r = engine.process(text)
        if isinstance(exp_intent, tuple):
            good = (r['intent'] in exp_intent)
        else:
            good = (r['intent'] == exp_intent)

        if exp_action is not None:
            good = good and (r['action'] == exp_action)
        if exp_target is not None:
            good = good and (r['target'] == exp_target)

        ok += good
        mark = 'OK ' if good else 'SAI'
        print(f'[{mark}] "{text}"\n      -> intent={r["intent"]}, action={r["action"]}, target={r["target"]}\n      "{r["text"]}"\n')

    print(f'=== Kết quả kiểm thử: {ok}/{total} đạt ===')
    return ok == total


def interactive():
    print('Gõ câu thoại thử nghiệm NLU (Ctrl+C để thoát):')
    while True:
        try:
            t = input('> ')
        except (EOFError, KeyboardInterrupt):
            break
        r = engine.process(t)
        if r:
            print(f'  [intent={r["intent"]}, action={r["action"]}, target={r["target"]}, cat={r["category"]}]')
            print(f'  ROBOT: {r["text"]}\n')


if __name__ == '__main__':
    if '-i' in sys.argv:
        interactive()
    else:
        success = run()
        sys.exit(0 if success else 1)
