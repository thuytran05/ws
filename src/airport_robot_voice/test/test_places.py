#!/usr/bin/env python3
"""
Unit tests kiểm tra tính hợp lệ của cơ sở dữ liệu places.json và intents.json.
"""

import json
import os
import pytest


@pytest.fixture(scope="module")
def places_data():
    cur_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_dir = os.path.dirname(cur_dir)
    places_file = os.path.join(pkg_dir, 'config', 'places.json')
    with open(places_file, 'r', encoding='utf-8') as f:
        return json.load(f)


@pytest.fixture(scope="module")
def intents_data():
    cur_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_dir = os.path.dirname(cur_dir)
    intents_file = os.path.join(pkg_dir, 'config', 'intents.json')
    with open(intents_file, 'r', encoding='utf-8') as f:
        return json.load(f)


class TestPlacesSchemaAndConsistency:
    def test_demo_flag_present(self, places_data):
        assert places_data.get('_demo_data') is True

    def test_required_gates_exist(self, places_data):
        places = places_data['places']
        for gate in ['gate_a1', 'gate_a2', 'gate_a3', 'gate_a4', 'gate_b1', 'gate_b2', 'gate_15']:
            assert gate in places, f"Thiếu cổng {gate} trong catalog"
            assert places[gate]['enabled'] is True
            assert isinstance(places[gate]['x'], (int, float))
            assert isinstance(places[gate]['y'], (int, float))
            assert isinstance(places[gate]['yaw'], (int, float))
            assert places[gate]['frame_id'] == 'map'

    def test_categories_and_multiple_toilets(self, places_data):
        places = places_data['places']
        categories = places_data.get('categories', {})

        assert 'toilet' in categories
        assert categories['toilet']['policy'] == 'shortest_path'

        toilets = [k for k, p in places.items() if p.get('category') == 'toilet']
        assert len(toilets) >= 2, f"Cần ít nhất 2 nhà vệ sinh trong catalog, tìm thấy: {toilets}"

    def test_no_duplicate_aliases_across_distinct_places(self, places_data):
        alias_to_place = {}
        places = places_data['places']
        for key, p in places.items():
            aliases = [key, p.get('name', '')] + p.get('aliases', [])
            for a in aliases:
                a_clean = str(a).lower().strip()
                if a_clean:
                    if a_clean in alias_to_place and alias_to_place[a_clean] != key:
                        pytest.fail(f"Alias '{a_clean}' bị trùng giữa {alias_to_place[a_clean]} và {key}")
                    alias_to_place[a_clean] = key

    def test_offline_services_present(self, places_data):
        services = places_data.get('services', {})
        assert 'wifi' in services
        assert 'Airport-Free-WiFi' in services['wifi']['name']
        assert 'charging' in services
        assert 'hours' in services


class TestIntentsSchema:
    def test_no_flight_query_in_intents(self, intents_data):
        # Theo đặc tả 5.2: Chức năng tra cứu chuyến bay của cấu hình cũ được loại khỏi runtime
        intent_names = [it['name'] for it in intents_data['intents']]
        assert 'ask_flight_status' not in intent_names

    def test_required_action_intents(self, intents_data):
        intents = {it['name']: it for it in intents_data['intents']}
        assert 'guide_me' in intents
        assert intents['guide_me']['action'] == 'navigate'

        assert 'pause_robot' in intents
        assert intents['pause_robot']['action'] == 'pause'

        assert 'resume_robot' in intents
        assert intents['resume_robot']['action'] == 'resume'

        assert 'cancel_robot' in intents
        assert intents['cancel_robot']['action'] == 'cancel'

        assert 'ask_status' in intents
        assert intents['ask_status']['action'] == 'status'

        assert 'replace_target' in intents
        assert intents['replace_target']['action'] == 'replace'
