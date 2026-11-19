#!/usr/bin/env python3
"""
nlu.py — Lõi hiểu ngôn ngữ tiếng Việt offline, thuần Python, không phụ thuộc LLM.
Xử lý:
- Tiếng Việt có dấu, không dấu, Unicode tổ hợp (NFC/NFD).
- Đọc số và mã cổng: 'a ba', 'bê hai', 'cổng số mười lăm', 'a tư'.
- Ranh giới từ trọn vẹn (word boundary): chống A123 bị cắt thành A12, alias A1 không khớp A10.
- Kiểm tra tính tồn tại của địa điểm trong danh mục trước khi phát lệnh điều hướng.
- Nhận diện phủ định: 'đừng dẫn...', 'không dừng lại' (không pause), 'không cần nữa' (cancel).
- Nhận diện câu đa đích (ambiguous): 'A1 hay A2' -> thông báo yêu cầu chưa rõ.
- Nhận diện câu thiếu đích: 'Dẫn tôi đến...' -> thông báo ngắn, kết thúc lượt, không pending slot.
- Phân biệt intent thông tin (wifi, sạc, giờ, chào, cảm ơn, help) và intent dẫn đường.
- Hỗ trợ chọn nhóm địa điểm (category: 'toilet') theo thuật toán đường đi.
"""

import difflib
import json
import math
import os
import re
import time
import unicodedata

# Dấu thanh điệu tiếng Việt (huyền, sắc, ngã, hỏi, nặng) trong Unicode tổ hợp (NFD)
_TONE_MARKS = {0x0300, 0x0301, 0x0303, 0x0309, 0x0323}


def strip_tones(text: str) -> str:
    """Xoá dấu thanh điệu, giữ nguyên âm gốc ô/ơ/ê/ư/ă/đ."""
    nfd = unicodedata.normalize('NFD', text)
    kept = ''.join(c for c in nfd if ord(c) not in _TONE_MARKS)
    return unicodedata.normalize('NFC', kept)


def strip_all(text: str) -> str:
    """Xoá toàn bộ dấu tiếng Việt (kể cả ô/ơ/ê/ư/ă, đ->d)."""
    text = text.replace('đ', 'd').replace('Đ', 'D')
    nfd = unicodedata.normalize('NFD', text)
    kept = ''.join(c for c in nfd if unicodedata.category(c) != 'Mn')
    return unicodedata.normalize('NFC', kept)


def normalize(text: str) -> str:
    """Chuẩn hóa chuỗi cơ bản: NFC, chữ thường, xoá ký tự đặc biệt, chuẩn khoảng trắng."""
    text = unicodedata.normalize('NFC', text or '').lower().strip()
    text = re.sub(r'[^\w\s]', ' ', text, flags=re.UNICODE)
    return re.sub(r'\s+', ' ', text).strip()


def is_accented(text: str) -> bool:
    """Kiểm tra chuỗi có ký tự tiếng Việt có dấu hay không."""
    return any(ord(c) > 127 for c in normalize(text))


def clean(text: str) -> str:
    """Dùng cho chấm điểm ý định: giữ nguyên âm gốc, xoá dấu thanh điệu."""
    return strip_tones(normalize(text))


def clean_ascii(text: str) -> str:
    """Dùng cho dò slot / tên địa điểm: xoá toàn bộ dấu để có dung sai cao."""
    return strip_all(normalize(text))


def expand_spoken_codes(text: str) -> str:
    """Chuyển các cách đọc mã cổng/số tiếng Việt thành dạng chuẩn hóa.
    VD: 'a ba' -> 'a3', 'bê hai' -> 'b2', 'mười lăm' -> '15', 'cổng số mười lăm' -> 'cổng 15'.
    """
    t = clean_ascii(text)
    # Ghép từ ngữ phức
    t = re.sub(r'\bmuoi\s+lam\b', '15', t)
    t = re.sub(r'\bmuoi\s+nam\b', '15', t)
    t = re.sub(r'\bmuoi\b', '10', t)
    t = re.sub(r'\bso\s+(\d+)\b', r'\1', t)

    # Chuyển chữ cái đọc cổng: a, b (be), c (ce/xe), d (de)
    t = re.sub(r'\bbe\b', 'b', t)
    t = re.sub(r'\bxe\b', 'c', t)
    t = re.sub(r'\bde\b', 'd', t)

    # Các từ số đơn lẻ
    for word, digit in [('khong', '0'), ('mot', '1'), ('hai', '2'), ('ba', '3'),
                        ('bon', '4'), ('tu', '4'), ('nam', '5'), ('lam', '5'),
                        ('sau', '6'), ('bay', '7'), ('tam', '8'), ('chin', '9')]:
        t = re.sub(rf'\b{word}\b', digit, t)

    # Ghép các cặp như 'a 3' -> 'a3', 'b 2' -> 'b2', 'c 1' -> 'c1', 'gate a 1' -> 'gate a1'
    t = re.sub(r'\b([a-d])\s+(\d{1,2})\b', r'\1\2', t)
    return t


_STOPWORDS_RAW = """
ơi anh chị em à ạ nhé nhỉ ha hả cho hỏi là gì có không được chưa
bạn tôi mình này đó kia thế nào đâu ở vậy giúp làm ơn xin một chút
đây thì và hay hoặc đến từ của với về các những rất quá ấy như nếu
sẽ đã đang rồi muốn cần nên phải luôn chỉ cũng cả mọi tất cả tớ
nha nè vâng dạ ừ trước sau ra vào lên xuống đi tới nơi chỗ cứ
"""
STOPWORDS = set(clean(w) for w in _STOPWORDS_RAW.split())
STOPWORDS_ASCII = set(clean_ascii(w) for w in _STOPWORDS_RAW.split())


def tokenize(text: str):
    return [w for w in clean(text).split() if w]


def tokenize_ascii(text: str):
    return [w for w in clean_ascii(text).split() if w]


def content_words(text: str):
    return [w for w in tokenize(text) if w not in STOPWORDS]


def content_words_ascii(text: str):
    return [w for w in tokenize_ascii(text) if w not in STOPWORDS_ASCII]


class NluEngine:
    def __init__(self, intents_file: str, places_file: str):
        with open(intents_file, 'r', encoding='utf-8') as f:
            self.cfg = json.load(f)
        with open(places_file, 'r', encoding='utf-8') as f:
            self.places_data = json.load(f)

        self.places = self.places_data.get('places', {})
        self.categories = self.places_data.get('categories', {})
        self.services = self.places_data.get('services', {})

        self.threshold = float(self.cfg.get('threshold', 35.0))
        self.priority_margin = float(self.cfg.get('priority_margin', 15.0))
        self.fallback = self.cfg.get('fallback_response',
                                     'Xin lỗi, tôi chưa hiểu ý bạn. Bạn có thể hỏi về cổng máy bay, quầy thủ tục hoặc nhà vệ sinh.')

        self._build_vocab()
        self._build_alias_map()

    def _build_vocab(self):
        intents = self.cfg['intents']
        n = len(intents)

        for it in intents:
            vocab, vocab_ascii = set(), set()
            for ex in it['examples']:
                vocab.update(content_words(ex))
                vocab_ascii.update(content_words_ascii(ex))
            for kw in it.get('boost_keywords', []):
                vocab.update(content_words(kw))
                vocab_ascii.update(content_words_ascii(kw))
            it['_vocab'] = vocab
            it['_vocab_ascii'] = vocab_ascii
            it['_boost_norm'] = [clean(k) for k in it.get('boost_keywords', [])]
            it['_boost_ascii'] = [clean_ascii(k) for k in it.get('boost_keywords', [])]

        df, df_ascii = {}, {}
        for it in intents:
            for w in it['_vocab']:
                df[w] = df.get(w, 0) + 1
            for w in it['_vocab_ascii']:
                df_ascii[w] = df_ascii.get(w, 0) + 1

        self.idf = {w: math.log((1 + n) / (1 + d)) + 1.0 for w, d in df.items()}
        self.idf_ascii = {w: math.log((1 + n) / (1 + d)) + 1.0 for w, d in df_ascii.items()}

    def _build_alias_map(self):
        self.alias_map = {}
        # Alias từ các địa điểm cụ thể
        for key, p in self.places.items():
            names = [key, p.get('name', '')] + p.get('aliases', [])
            for name in names:
                norm_name = clean_ascii(str(name))
                if norm_name:
                    self.alias_map[norm_name] = {'type': 'place', 'key': key}

        # Alias từ danh mục
        for cat_key, cat in self.categories.items():
            names = [cat_key, cat.get('name', '')] + cat.get('aliases', [])
            for name in names:
                norm_name = clean_ascii(str(name))
                if norm_name:
                    self.alias_map[norm_name] = {'type': 'category', 'key': cat_key}

    def _word_weight(self, w, ascii_mode=False):
        table = self.idf_ascii if ascii_mode else self.idf
        return table.get(w, 1.0)

    def _in_vocab(self, word, vocab):
        if word in vocab:
            return True
        if len(word) < 5:
            return False
        for v in vocab:
            if len(v) < 5 or abs(len(v) - len(word)) > 2:
                continue
            if difflib.SequenceMatcher(None, word, v).ratio() >= 0.88:
                return True
        return False

    def _score_one(self, query_words, vocab, ascii_mode):
        if not query_words:
            return 0.0
        matched = sum(self._word_weight(w, ascii_mode) for w in query_words
                      if self._in_vocab(w, vocab))
        total = sum(self._word_weight(w, ascii_mode) for w in query_words)
        return 100.0 * matched / total if total else 0.0

    def score_intent(self, words, norm_query, it, ascii_mode):
        vocab = it['_vocab_ascii'] if ascii_mode else it['_vocab']
        boost = it['_boost_ascii'] if ascii_mode else it['_boost_norm']

        base = self._score_one(words, vocab, ascii_mode)
        matched_boost = [kw for kw in boost if kw and kw in norm_query]
        if matched_boost:
            max_boost_len = max(len(kw.split()) for kw in matched_boost)
            base = max(base, 90.0) + (max_boost_len * 5.0)
        return base

    def match_intent(self, raw: str):
        ascii_mode = not is_accented(raw)
        norm = clean_ascii(raw) if ascii_mode else clean(raw)
        words = content_words_ascii(raw) if ascii_mode else content_words(raw)

        scored = [(it, self.score_intent(words, norm, it, ascii_mode))
                  for it in self.cfg['intents']]
        passed = [(it, s) for it, s in scored if s >= self.threshold]
        pool = passed if passed else scored

        pool_sorted = sorted(pool, key=lambda p: p[1], reverse=True)
        best, best_score = pool_sorted[0]

        close = [(it, s) for it, s in pool_sorted
                 if best_score - s <= self.priority_margin]
        if len(close) > 1:
            best, best_score = max(close, key=lambda p: (int(p[0].get('priority', 1)), p[1]))

        return best, best_score

    # ------------------------------------------------------------------
    # Trích xuất địa điểm, danh mục và phát hiện đa đích (ambiguity)
    # ------------------------------------------------------------------
    def extract_all_targets(self, raw: str):
        """Trích xuất toàn bộ đích được đề cập trong câu:
        - Dùng ranh giới từ r'\b' chặt chẽ.
        - Trả về danh sách dict: [{'type': 'place'|'category', 'key': ..., 'matched_text': ...}]
        """
        expanded = expand_spoken_codes(raw)
        found_targets = []
        seen_keys = set()
        matched_spans = []

        # 1. Tìm cổng theo mã quy chuẩn: A1, A2, A3, A4, B1, B2, C1...
        gate_pattern = r'\b(?:cong|cua|gate)?\s*([a-d])(\d{1,2})\b'
        for m in re.finditer(gate_pattern, expanded):
            letter = m.group(1).lower()
            num = m.group(2)
            gate_code = f'{letter.upper()}{num}'
            place_key = f'gate_{letter}{num}'
            matched_spans.append(m.span())
            if place_key not in seen_keys:
                seen_keys.add(place_key)
                found_targets.append({
                    'type': 'place',
                    'key': place_key,
                    'gate_code': gate_code,
                    'exists': place_key in self.places,
                    'matched_text': m.group(0)
                })

        # Cổng chỉ có số: ví dụ 'cổng 15', 'cửa 15', 'gate 15'
        num_gate_pattern = r'\b(?:cong|cua|gate)\s+(\d{1,2})\b'
        for m in re.finditer(num_gate_pattern, expanded):
            num = m.group(1)
            gate_code = num
            place_key = f'gate_{num}'
            matched_spans.append(m.span())
            if place_key not in seen_keys:
                seen_keys.add(place_key)
                found_targets.append({
                    'type': 'place',
                    'key': place_key,
                    'gate_code': gate_code,
                    'exists': place_key in self.places,
                    'matched_text': m.group(0)
                })

        # 2. Tìm theo alias map (sắp xếp dài trước ngắn sau, khớp theo token ranh giới từ)
        for alias, info in sorted(self.alias_map.items(), key=lambda x: len(x[0]), reverse=True):
            pattern = rf'\b{re.escape(alias)}\b'
            for m in re.finditer(pattern, expanded):
                span = m.span()
                # Nếu span này nằm bên trong một span dài hơn đã khớp trước, bỏ qua
                if any(ex[0] <= span[0] and span[1] <= ex[1] for ex in matched_spans):
                    continue
                matched_spans.append(span)
                k = info['key']
                if k not in seen_keys:
                    seen_keys.add(k)
                    found_targets.append({
                        'type': info['type'],
                        'key': k,
                        'exists': (k in self.places if info['type'] == 'place' else k in self.categories),
                        'matched_text': alias
                    })

        # 3. Kiểm tra các số cổng không hợp lệ / ngoài tầm (ví dụ A123, B99...)
        invalid_gate_pattern = r'\b(?:cong|cua|gate)?\s*([a-z])(\d{3,})\b'
        for m in re.finditer(invalid_gate_pattern, expanded):
            bad_code = f'{m.group(1).upper()}{m.group(2)}'
            bad_key = f'gate_{m.group(1).lower()}{m.group(2)}'
            matched_spans.append(m.span())
            if bad_key not in seen_keys:
                seen_keys.add(bad_key)
                found_targets.append({
                    'type': 'place',
                    'key': bad_key,
                    'gate_code': bad_code,
                    'exists': False,
                    'matched_text': m.group(0)
                })

        # 4. Nếu có cả điểm cụ thể và danh mục cha của điểm đó (ví dụ toilet_1 và toilet), giữ lại điểm cụ thể
        specific_cats = {self.places[t['key']].get('category') for t in found_targets if t['type'] == 'place' and t['key'] in self.places}
        filtered = [t for t in found_targets if not (t['type'] == 'category' and t['key'] in specific_cats)]
        return filtered

    # ------------------------------------------------------------------
    # Kiểm tra phủ định
    # ------------------------------------------------------------------
    def check_negation(self, raw: str):
        """Phân biệt phủ định:
        - Cancel: 'không cần nữa', 'không đi nữa', 'thôi không đi', 'hủy chuyến' -> True, 'cancel'
        - Negated pause: 'không dừng lại', 'đừng dừng' -> True, 'negated_pause'
        - Negated navigate: 'đừng dẫn', 'không dẫn', 'đừng đưa', 'không đi cổng a1' -> True, 'negated_nav'
        """
        norm = clean_ascii(raw)

        # Lệnh hủy (Cancel)
        if re.search(r'\b(khong\s+can\s+nua|khong\s+di\s+nua|thoi\s+khong\s+di|huy\s+chuyen|huy\s+nhiem\s+vu|khong\s+can\s+dan|khong\s+can)\b', norm):
            return True, 'cancel'

        # Phủ định dừng lại (Không được pause!)
        if re.search(r'\b(khong|dung|cho)\s+(dung|dung\s+lai|pause|cho)\b', norm):
            return True, 'negated_pause'

        # Phủ định dẫn đường
        if re.search(r'\b(dung|khong|cho)\s+(dan|dua|chi\s+duong|di\s+toi|di\s+den|di\s+theo)\b', norm):
            return True, 'negated_nav'
        if re.search(r'\bkhong\s+di\b', norm):
            return True, 'negated_nav'

        return False, None

    # ------------------------------------------------------------------
    # Xử lý toàn diện câu thoại
    # ------------------------------------------------------------------
    def process(self, raw: str):
        raw = (raw or '').strip()
        if not raw:
            return None

        slots = {}

        # 1. Kiểm tra phủ định trước
        is_neg, neg_type = self.check_negation(raw)
        if is_neg:
            if neg_type == 'cancel':
                return {
                    'raw': raw,
                    'text': 'Tôi đã hủy chuyến đi hiện tại.',
                    'intent': 'cancel_robot',
                    'score': 100.0,
                    'action': 'cancel',
                    'target': None,
                    'category': None,
                    'selection_policy': None,
                    'slots': slots,
                    'is_valid_target': False
                }
            elif neg_type == 'negated_pause':
                return {
                    'raw': raw,
                    'text': 'Tôi tiếp tục di chuyển và không dừng lại.',
                    'intent': 'negated_pause',
                    'score': 90.0,
                    'action': None,
                    'target': None,
                    'category': None,
                    'selection_policy': None,
                    'slots': slots,
                    'is_valid_target': False
                }
            elif neg_type == 'negated_nav':
                return {
                    'raw': raw,
                    'text': 'Tôi đã ghi nhận bạn không muốn dẫn đường lúc này.',
                    'intent': 'negated_nav',
                    'score': 95.0,
                    'action': None,
                    'target': None,
                    'category': None,
                    'selection_policy': None,
                    'slots': slots,
                    'is_valid_target': False
                }

        # 2. Trích xuất các đích đến
        targets = self.extract_all_targets(raw)

        # 3. Kiểm tra đa đích (Ambiguous)
        if len(targets) > 1:
            return {
                'raw': raw,
                'text': 'Yêu cầu có nhiều điểm đến chưa rõ ràng. Bạn vui lòng chọn một điểm.',
                'intent': 'ambiguous_target',
                'score': 90.0,
                'action': None,
                'target': None,
                'category': None,
                'selection_policy': None,
                'slots': {'candidates': [t['key'] for t in targets]},
                'is_valid_target': False
            }

        # 4. So khớp intent
        intent, score = self.match_intent(raw)

        # 5. Phân loại theo kết quả intent & target
        if intent is None or score < self.threshold:
            return {
                'raw': raw,
                'text': self.fallback,
                'intent': 'unknown',
                'score': score,
                'action': None,
                'target': None,
                'category': None,
                'selection_policy': None,
                'slots': slots,
                'is_valid_target': False
            }

        action = intent.get('action')
        intent_name = intent['name']

        # Xử lý các lệnh điều khiển chuyến (pause, resume, cancel, status, help)
        if action in ('pause', 'resume', 'cancel', 'status', 'help'):
            return {
                'raw': raw,
                'text': intent['responses'][0],
                'intent': intent_name,
                'score': score,
                'action': action,
                'target': None,
                'category': None,
                'selection_policy': None,
                'slots': slots,
                'is_valid_target': False
            }

        # Xử lý câu chào, cảm ơn, thời gian, wifi, tiện ích (action is None)
        if action is None:
            resp = intent['responses'][0]
            if '{time}' in resp:
                resp = resp.replace('{time}', time.strftime('%H giờ %M phút'))
            return {
                'raw': raw,
                'text': resp,
                'intent': intent_name,
                'score': score,
                'action': None,
                'target': None,
                'category': None,
                'selection_policy': None,
                'slots': slots,
                'is_valid_target': False
            }

        # Xử lý hành động điều hướng (action == 'navigate')
        target_info = targets[0] if targets else None

        # Nếu intent đã chỉ định sẵn place hoặc category (ví dụ ask_checkin -> checkin, ask_toilet -> toilet, go_home -> home)
        if not target_info and intent.get('place'):
            target_info = {'type': 'place', 'key': intent['place'], 'exists': intent['place'] in self.places}
        elif not target_info and intent.get('category'):
            target_info = {'type': 'category', 'key': intent['category'], 'exists': intent['category'] in self.categories}

        # Nếu không có đích nào
        if not target_info:
            return {
                'raw': raw,
                'text': 'Tôi chưa xác định được điểm đến. Bạn vui lòng nói rõ điểm bạn muốn đến.',
                'intent': 'missing_destination',
                'score': score,
                'action': None,
                'target': None,
                'category': None,
                'selection_policy': None,
                'slots': slots,
                'is_valid_target': False
            }

        # Kiểm tra tính tồn tại của đích trong danh mục
        if not target_info.get('exists'):
            raw_code = target_info.get('gate_code') or target_info.get('matched_text') or target_info.get('key')
            return {
                'raw': raw,
                'text': f'Địa điểm {raw_code} chưa có trong danh mục của tôi.',
                'intent': 'place_not_found',
                'score': score,
                'action': None,
                'target': None,
                'category': None,
                'selection_policy': None,
                'slots': {'attempted': target_info.get('key')},
                'is_valid_target': False
            }

        # Đích là danh mục nhiều điểm (ví dụ toilet)
        if target_info['type'] == 'category':
            cat_key = target_info['key']
            cat_obj = self.categories.get(cat_key, {})
            slots['category'] = cat_key
            policy = cat_obj.get('policy', 'shortest_path')
            return {
                'raw': raw,
                'text': f'Tôi sẽ tìm đường tới {cat_obj.get("name", "địa điểm")} phù hợp nhất cho bạn.',
                'intent': intent_name,
                'score': score,
                'action': 'navigate',
                'target': None,
                'category': cat_key,
                'selection_policy': policy,
                'slots': slots,
                'is_valid_target': True
            }

        # Đích là một địa điểm cụ thể
        target_key = target_info['key']
        place = self.places.get(target_key, {})
        place_name = place.get('name', target_key)
        slots['place_key'] = target_key
        if 'gate_code' in target_info:
            slots['gate'] = target_info['gate_code']

        resp = f'Tôi sẽ dẫn bạn đến {place_name}.'
        return {
            'raw': raw,
            'text': resp,
            'intent': intent_name,
            'score': score,
            'action': 'navigate',
            'target': target_key,
            'category': place.get('category'),
            'selection_policy': 'direct',
            'slots': slots,
            'is_valid_target': True
        }
