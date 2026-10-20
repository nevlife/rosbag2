# Copyright 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from pathlib import Path

import pytest

from rosbag2_py import bag_rewrite, StorageOptions
from rosbag2_test_common import TESTED_STORAGE_IDS
import yaml

RESOURCES_PATH = Path(os.environ['ROSBAG2_PY_TEST_RESOURCES_DIR'])


def test_no_toplevel_key(tmpdir):
    output_options_path = tmpdir / 'no_toplevel_key.yml'
    output_options_content = '[{key: value}]'
    with output_options_path.open('w') as f:
        f.write(output_options_content)
    with pytest.raises(RuntimeError):
        bag_rewrite([], str(output_options_path))


def test_output_bags_not_a_list(tmpdir):
    output_options_path = tmpdir / 'not_a_list.yml'
    output_options_content = '{output_bags: {key: value}}'
    with output_options_path.open('w') as f:
        f.write(output_options_content)
    with pytest.raises(RuntimeError):
        bag_rewrite([], str(output_options_path))


@pytest.mark.parametrize('storage_id', TESTED_STORAGE_IDS)
def test_basic_convert(tmpdir, storage_id):
    # This test is just to test that the rosbag2_py wrapper parses input
    # It is not a comprehensive test of bag_rewrite.
    bag_a_path = RESOURCES_PATH / storage_id / 'convert_a'
    bag_b_path = RESOURCES_PATH / storage_id / 'convert_b'
    output_uri_1 = tmpdir / storage_id / 'converted_1'
    output_uri_2 = tmpdir / storage_id / 'converted_2'
    input_options = [
        StorageOptions(uri=str(bag_a_path)),
        StorageOptions(uri=str(bag_b_path)),
    ]
    output_options_path = tmpdir / 'simple_convert.yml'
    output_options_content = f"""
output_bags:
- uri: {output_uri_1}
  storage_id: {storage_id}
  topics: [a_empty]
- uri: {output_uri_2}
  storage_id: {storage_id}
  exclude: ".*empty.*"
"""
    with output_options_path.open('w') as f:
        f.write(output_options_content)
    bag_rewrite(input_options, str(output_options_path))
    assert output_uri_1.exists()
    assert output_uri_1.isdir()
    assert (output_uri_1 / 'metadata.yaml').exists()

    assert output_uri_2.exists()
    assert output_uri_2.isdir()
    assert (output_uri_2 / 'metadata.yaml').exists()


@pytest.mark.parametrize('storage_id', TESTED_STORAGE_IDS)
def test_convert_inclusive_message_ranges(tmpdir, storage_id):
    """Keep inclusive first and last message indices through YAML conversion."""
    output = tmpdir / 'ranged'
    config = tmpdir / 'ranged.yaml'
    config.write(f"""
output_bags:
- uri: {output}
  storage_id: {storage_id}
  all_topics: true
  topic_message_ranges:
    a_empty: {{start: 0, end: 0}}
    b_basictypes: {{start: 48, end: 49}}
""")
    bag_rewrite([StorageOptions(uri=str(RESOURCES_PATH / storage_id / 'convert_a'))],
                str(config))
    with (output / 'metadata.yaml').open() as stream:
        metadata = yaml.safe_load(stream)['rosbag2_bagfile_information']
    counts = {topic['topic_metadata']['name']: topic['message_count']
              for topic in metadata['topics_with_message_count']}
    assert counts == {'a_empty': 1, 'b_basictypes': 2}


@pytest.mark.parametrize('bounds', [
    '{start: 2, end: 1}', '{start: -1, end: 1}', '{start: 0, end: 100}',
    '{start: 1}', '{end: 1}', '[]'])
def test_convert_rejects_invalid_message_ranges(tmpdir, bounds):
    """Reject invalid YAML ranges before creating an output bag."""
    output = tmpdir / 'invalid'
    config = tmpdir / 'invalid.yaml'
    config.write(f"""
output_bags:
- uri: {output}
  storage_id: mcap
  all_topics: true
  topic_message_ranges:
    a_empty: {bounds}
""")
    with pytest.raises((ValueError, RuntimeError)):
        bag_rewrite([StorageOptions(uri=str(RESOURCES_PATH / 'mcap' / 'convert_a'))],
                    str(config))
    assert not output.exists()
