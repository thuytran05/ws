#!/usr/bin/env python3
"""
Unit tests cho lõi NLU tiếng Việt của airport_robot_voice.
Chạy độc lập bằng pytest hoặc python3.
"""

import os
import unicodedata
import pytest

from airport_robot_voice.nlu import NluEngine


@pytest.fixture(scope="module")
def engine():
    # Tìm file config từ thư mục config của package
    cur_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_dir = os.path.dirname(cur_dir)
    cfg_dir = os.path.join(pkg_dir, 'config')

    intents_file = os.path.join(cfg_dir, 'intents.json')
    places_file = os.path.join(cfg_dir, 'places.json')
    return NluEngine(intents_file, places_file)


class TestNluBasicAndGates:
    def test_gate_a3_navigation(self, engine):
        r = engine.process("Dẫn tôi đến cổng A3")
        assert r['action'] == 'navigate'
        assert r['target'] == 'gate_a3'
        assert r['is_valid_target'] is True

    def test_gate_location_triggers_navigation(self, engine):
        # Đặc tả 5.1 & 5.2: Hỏi vị trí của địa điểm hợp lệ chuyển sang dẫn đường
        r = engine.process("Cổng A3 ở đâu")
        assert r['action'] == 'navigate'
        assert r['target'] == 'gate_a3'

        r2 = engine.process("Cổng a ba ở chỗ nào")
        assert r2['action'] == 'navigate'
        assert r2['target'] == 'gate_a3'

    def test_spoken_code_variants(self, engine):
        # Bê hai -> gate_b2
        r = engine.process("Cho tôi tới cổng bê hai")
        assert r['action'] == 'navigate'
        assert r['target'] == 'gate_b2'

        # Cổng số mười lăm -> gate_15
        r2 = engine.process("Cổng số mười lăm ở đâu")
        assert r2['action'] == 'navigate'
        assert r2['target'] == 'gate_15'

        # A bốn / a tư -> gate_a4
        r3 = engine.process("Dẫn tôi tới cổng a tư")
        assert r3['action'] == 'navigate'
        assert r3['target'] == 'gate_a4'

    def test_nonexistent_gate_detected(self, engine):
        # Mã cổng không tồn tại -> báo lỗi trước khi phát lệnh
        r = engine.process("Dẫn tôi đến cổng A99")
        assert r['action'] is None
        assert r['intent'] == 'place_not_found'
        assert "A99" in r['text'] or "chưa có" in r['text']

    def test_word_boundary_prevents_partial_match(self, engine):
        # A123 không được cắt thành A12
        r = engine.process("Dẫn tôi đến cổng A123")
        assert r['target'] != 'gate_a1'
        assert r['target'] != 'gate_a2'
        assert r['action'] is None

    def test_alias_a1_does_not_match_a10(self, engine):
        r = engine.process("Dẫn tôi đến cổng A10")
        assert r['target'] != 'gate_a1'


class TestCategoriesAndPlaces:
    def test_checkin_navigation(self, engine):
        r = engine.process("Cho tôi tới quầy check-in")
        assert r['action'] == 'navigate'
        assert r['target'] == 'checkin'

    def test_toilet_category(self, engine):
        # Nhà vệ sinh thuộc nhóm category
        r = engine.process("Dẫn tôi đến nhà vệ sinh")
        assert r['action'] == 'navigate'
        assert r['category'] == 'toilet'
        assert r['selection_policy'] == 'shortest_path'

    def test_specific_toilet(self, engine):
        # Chỉ định rõ sảnh A
        r = engine.process("Dẫn tôi đến nhà vệ sinh sảnh A")
        assert r['action'] == 'navigate'
        assert r['target'] == 'toilet_1'


class TestNegationsAndControls:
    def test_negated_navigation(self, engine):
        # Đừng dẫn tôi -> không phát sinh lệnh dẫn đường
        r = engine.process("Đừng dẫn tôi đến cổng A1")
        assert r['action'] is None
        assert r['intent'] == 'negated_nav'

    def test_cancel_phrases(self, engine):
        r = engine.process("Không cần nữa")
        assert r['action'] == 'cancel'

        r2 = engine.process("Hủy chuyến đi giúp tôi")
        assert r2['action'] == 'cancel'

    def test_negated_pause_not_triggering_pause(self, engine):
        # "không dừng lại" không được biến thành pause
        r = engine.process("Không dừng lại nhé robot")
        assert r['action'] != 'pause'

    def test_pause_commands(self, engine):
        r = engine.process("Dừng lại")
        assert r['action'] == 'pause'

        r2 = engine.process("Chờ tôi một chút")
        assert r2['action'] == 'pause'

    def test_resume_commands(self, engine):
        r = engine.process("Đi tiếp nào")
        assert r['action'] == 'resume'


class TestAmbiguityAndMissing:
    def test_ambiguous_multiple_destinations(self, engine):
        # Nhiều đích trong một câu -> báo yêu cầu chưa rõ ràng
        r = engine.process("Dẫn tôi đến cổng A1 hay A2")
        assert r['action'] is None
        assert r['intent'] == 'ambiguous_target'

    def test_missing_destination(self, engine):
        # Dẫn tôi đến... không có đích -> kết thúc lượt, không pending
        r = engine.process("Dẫn tôi đến")
        assert r['action'] is None
        assert r['intent'] == 'missing_destination'


class TestOfflineInformation:
    def test_wifi_query(self, engine):
        r = engine.process("Wifi sân bay tên gì")
        assert r['action'] is None
        assert "Airport-Free-WiFi" in r['text']

    def test_charging_query(self, engine):
        r = engine.process("Có chỗ sạc điện thoại không")
        assert r['action'] is None
        assert "sạc" in r['text'].lower()

    def test_hours_query(self, engine):
        r = engine.process("Sân bay mở cửa mấy giờ")
        assert r['action'] is None
        assert "24/7" in r['text']

    def test_greetings_and_thanks(self, engine):
        r = engine.process("Xin chào robot")
        assert r['intent'] == 'greeting'
        assert r['action'] is None

        r2 = engine.process("Cảm ơn bạn nhiều")
        assert r2['intent'] == 'thanks'
        assert r2['action'] is None

    def test_status_query(self, engine):
        r = engine.process("Bạn đang dẫn tôi đi đâu đấy")
        assert r['action'] == 'status'

    def test_out_of_scope(self, engine):
        r = engine.process("Hôm nay thời tiết thế nào bạn ơi")
        assert r['intent'] == 'unknown'
        assert r['action'] is None


class TestUnicodeAndNoDiacritics:
    def test_decomposed_unicode(self, engine):
        nfd_text = unicodedata.normalize('NFD', "Dẫn tôi đến cổng A3")
        r = engine.process(nfd_text)
        assert r['action'] == 'navigate'
        assert r['target'] == 'gate_a3'

    def test_no_diacritics_input(self, engine):
        r = engine.process("dan toi den cong a3")
        assert r['action'] == 'navigate'
        assert r['target'] == 'gate_a3'

        r2 = engine.process("toilet o dau")
        assert r2['action'] == 'navigate'
        assert r2['category'] == 'toilet'


class TestNluReplaceDestination:
    def test_replace_gate_phrases(self, engine):
        for phrase in [
            "đổi sang cổng A4",
            "đổi điểm đến sang cổng A4",
            "chuyển sang cổng A4",
            "đổi đích sang cổng a4",
            "đổi lộ trình sang cổng A4"
        ]:
            r = engine.process(phrase)
            assert r['action'] == 'replace', f"Failed on '{phrase}': action={r.get('action')}"
            assert r['target'] == 'gate_a4', f"Failed on '{phrase}': target={r.get('target')}"
            assert r['is_valid_target'] is True
            assert 'Đã đổi lộ trình' in r['text']

    def test_replace_category_phrases(self, engine):
        for phrase in ["đổi sang nhà vệ sinh", "chuyển sang nhà vệ sinh", "đổi điểm đến sang nhà vệ sinh"]:
            r = engine.process(phrase)
            assert r['action'] == 'replace', f"Failed on '{phrase}': action={r.get('action')}"
            assert r['category'] == 'toilet', f"Failed on '{phrase}': category={r.get('category')}"
            assert r['is_valid_target'] is True
            assert 'Đã đổi lộ trình' in r['text']

    def test_replace_nonexistent_gate(self, engine):
        r = engine.process("đổi sang cổng A99")
        assert r['intent'] == 'place_not_found'
        assert r['action'] is None

    def test_replace_missing_destination(self, engine):
        r = engine.process("đổi sang")
        assert r['intent'] == 'missing_destination'
        assert r['action'] is None
