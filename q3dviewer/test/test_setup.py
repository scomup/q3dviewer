import runpy
import unittest
from pathlib import Path
from unittest.mock import patch


class CUDAArchitectureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        setup_path = Path(__file__).resolve().parents[2] / 'setup.py'
        with patch('setuptools.setup'):
            build_config = runpy.run_path(str(setup_path))
        cls.get_arch_flags = staticmethod(build_config['get_cuda_arch_flags'])

    def test_only_supported_targets_are_selected(self):
        cases = [
            ('sm_35 sm_60 sm_61 sm_70 sm_75', ('60', '61', '70', '75')),
            ('sm_60 sm_61 sm_70 sm_75 sm_80 sm_86 sm_87',
             ('60', '61', '70', '75', '80', '86')),
            ('sm_60 sm_61 sm_70 sm_75 sm_80 sm_86 sm_89 sm_90',
             ('60', '61', '70', '75', '80', '86', '89')),
            ('sm_75 sm_80 sm_86 sm_89 sm_90', ('75', '80', '86', '89')),
        ]
        for supported, expected in cases:
            with self.subTest(supported=supported):
                with patch('subprocess.check_output', return_value=supported) as query:
                    flags = self.get_arch_flags('/cuda/bin/nvcc')
                query.assert_called_once_with(
                    ['/cuda/bin/nvcc', '--list-gpu-code'], text=True)
                self.assertEqual(flags, [
                    f'-gencode=arch=compute_{arch},code=sm_{arch}'
                    for arch in expected
                ] + [f'-gencode=arch=compute_{expected[-1]},code=compute_{expected[-1]}'])

    def test_no_supported_targets_reports_clear_error(self):
        for supported in ('', 'sm_35 sm_50'):
            with self.subTest(supported=supported):
                with patch('subprocess.check_output', return_value=supported):
                    with self.assertRaisesRegex(RuntimeError, 'configured GPU targets'):
                        self.get_arch_flags('/cuda/bin/nvcc')


if __name__ == '__main__':
    unittest.main()