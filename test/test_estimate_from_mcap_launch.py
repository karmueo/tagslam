# Copyright 2026 TagSLAM contributors
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

import importlib.util
from pathlib import Path

from launch import LaunchContext
import pytest


MODULE_PATH = (Path(__file__).resolve().parents[1] /
               'launch/estimate_from_mcap.launch.py')
spec = importlib.util.spec_from_file_location('estimate_from_mcap_launch',
                                              MODULE_PATH)
pipeline_launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pipeline_launch)


def test_creates_missing_directory(tmp_path):
    output = tmp_path / 'new_run'

    pipeline_launch.prepare_output_directory(output, overwrite=False)

    assert output.is_dir()
    assert not any(output.iterdir())


def test_replaces_existing_directory(tmp_path):
    output = tmp_path / 'old_run'
    (output / 'config').mkdir(parents=True)
    (output / 'config' / 'cameras.yaml').write_text('stale\n', encoding='utf-8')
    (output / 'trajectory.txt').write_text('stale\n', encoding='utf-8')
    input_file = tmp_path / 'old_run.mcap'
    input_file.write_bytes(b'input MCAP')

    pipeline_launch.prepare_output_directory(
        output, overwrite=True, input_files=(input_file,))

    assert output.is_dir()
    assert not any(output.iterdir())
    assert input_file.read_bytes() == b'input MCAP'


def test_rejects_existing_directory_without_overwrite(tmp_path):
    output = tmp_path / 'old_run'
    output.mkdir()
    (output / 'keep.txt').write_text('data\n', encoding='utf-8')

    with pytest.raises(RuntimeError, match='already exists'):
        pipeline_launch.prepare_output_directory(output, overwrite=False)

    assert (output / 'keep.txt').is_file()


def test_rejects_existing_file(tmp_path):
    output = tmp_path / 'run.mcap'
    output.write_text('data\n', encoding='utf-8')

    with pytest.raises(RuntimeError, match='not a directory'):
        pipeline_launch.prepare_output_directory(output, overwrite=True)

    assert output.is_file()


def test_rejects_directory_containing_working_directory(tmp_path, monkeypatch):
    output = tmp_path / 'workspace'
    nested = output / 'nested'
    nested.mkdir(parents=True)
    (nested / 'keep.txt').write_text('data\n', encoding='utf-8')
    monkeypatch.chdir(nested)

    with pytest.raises(RuntimeError, match='working directory'):
        pipeline_launch.prepare_output_directory(output, overwrite=True)

    assert (nested / 'keep.txt').is_file()


@pytest.mark.parametrize('nested', [False, True])
@pytest.mark.parametrize('contained_input', [
    'in_bag', 'calib', 'default_calib', 'default_calib_symlink',
])
def test_launch_rejects_output_containing_inputs(
        tmp_path, monkeypatch, nested, contained_input):
    output = tmp_path / 'output'
    input_directory = output / 'nested' / 'inputs' if nested else output
    input_directory.mkdir(parents=True)
    external = tmp_path / 'external'
    external.mkdir()
    in_bag = (input_directory if contained_input in ('in_bag', 'default_calib')
              else external) / 'input.mcap'
    calib = (external if contained_input == 'in_bag'
             else input_directory) / 'calib.yaml'
    in_bag.write_bytes(b'original MCAP')
    calib.write_bytes(b'original calibration')
    if contained_input == 'default_calib_symlink':
        (external / 'calib.yaml').symlink_to(calib)
    calib_arg = ('' if contained_input.startswith('default_calib')
                 else str(calib))
    marker = output / 'keep.txt'
    marker.write_bytes(b'previous output')
    monkeypatch.chdir(tmp_path)
    context = LaunchContext()
    context.launch_configurations.update({
        'in_bag': str(in_bag), 'calib': calib_arg,
        'output_dir': str(output), 'image_topic': '/stereo/image_raw',
        'vio_odom_topic': '', 'minimum_tags_for_pose': '1',
        'keep_intermediate': 'false',
    })

    with pytest.raises(RuntimeError, match='contains input file'):
        pipeline_launch.launch_setup(context)

    assert in_bag.read_bytes() == b'original MCAP'
    assert calib.read_bytes() == b'original calibration'
    assert marker.read_bytes() == b'previous output'


@pytest.mark.parametrize('alias', ['output', 'input'])
def test_rejects_symlink_alias_to_input_directory(tmp_path, alias):
    output = tmp_path / 'output'
    output.mkdir()
    input_file = output / 'input.mcap'
    input_file.write_bytes(b'original MCAP')
    link = tmp_path / 'alias'
    link.symlink_to(output if alias == 'output' else input_file)

    with pytest.raises(RuntimeError, match='contains input file'):
        pipeline_launch.prepare_output_directory(
            link if alias == 'output' else output, overwrite=True,
            input_files=(link if alias == 'input' else input_file,))

    assert input_file.read_bytes() == b'original MCAP'
