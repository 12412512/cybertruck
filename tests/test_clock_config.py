"""时钟配置负例：拒绝旧HSI、错误晶振值和启动失败后继续初始化。"""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import check_project as project


class ClockTests(unittest.TestCase):
    def setUp(self):
        self.reader = project.read
        self.data = project.load_ioc()
        project.ERRORS.clear()

    def check_source(self, changes=None):
        def read(path):
            text = self.reader(path)
            for name, old, new in changes or []:
                if path.name == name:
                    self.assertIn(old, text)
                    text = text.replace(old, new)
            return text
        with patch.object(project, 'read', read):
            project.verify_sources(self.data)
        return project.ERRORS.copy()

    def test_current_ioc_and_generated_source_agree(self):
        project.verify_ioc(self.data)
        self.assertEqual(self.check_source(), [])

    def test_old_clock_and_bypass_ioc_rejected(self):
        for key, value in [('RCC.PLLSourceVirtual', 'RCC_PLLSOURCE_HSI'),
                           ('RCC.HSE_VALUE', '25000000'), ('RCC.PLLM', '16'),
                           ('PH0-OSC_IN.Mode', 'HSE-External-Clock-Source')]:
            with self.subTest(key=key):
                project.ERRORS.clear()
                project.verify_ioc(dict(self.data, **{key: value}))
                self.assertTrue(project.ERRORS)

    def test_wrong_header_or_generated_clock_rejected(self):
        cases = [('stm32f4xx_hal_conf.h', '8000000U', '25000000U'),
                 ('main.c', 'RCC_PLLSOURCE_HSE', 'RCC_PLLSOURCE_HSI'),
                 ('main.c', 'RCC_HSE_ON', 'RCC_HSE_BYPASS')]
        for case in cases:
            with self.subTest(case=case):
                project.ERRORS.clear()
                self.assertTrue(self.check_source([case]))

    def test_oscillator_or_clock_failure_must_not_continue(self):
        for call in ('HAL_RCC_OscConfig(&RCC_OscInitStruct)',
                     'HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3)'):
            with self.subTest(call=call):
                project.ERRORS.clear()
                self.assertTrue(self.check_source([('main.c', call + ' != HAL_OK',
                                                    call + ' == HAL_OK')]))

    def test_failure_handler_must_not_return(self):
        self.assertTrue(self.check_source([('main.c', 'while (1)', 'if (1)')]))

    def test_can_must_not_start_before_clock(self):
        self.assertTrue(self.check_source([('main.c', 'SystemClock_Config();',
                                           'MX_CAN1_Init();\nSystemClock_Config();')]))

    def test_derived_rates_preserve_can_pwm_and_systick(self):
        # 独立重算，不能只相信IOC保存的显示频率。
        d = self.data
        vco = int(d['RCC.HSE_VALUE']) / int(d['RCC.PLLM']) * int(d['RCC.PLLN'])
        sysclk, apb1, apb2 = vco / 2, vco / 8, vco / 4
        self.assertEqual((sysclk, apb1, apb2, vco / int(d['RCC.PLLQ'])),
                         (120000000, 30000000, 60000000, 48000000))
        self.assertEqual(apb1 / int(d['CAN1.Prescaler']) / (1 + 12 + 2), 500000)
        self.assertEqual(apb2 * 2 / (int(d['TIM1.Prescaler']) + 1) /
                         (int(d['TIM1.Period']) + 1), 50)
        self.assertEqual(sysclk / 1000 - 1, 119999)


if __name__ == '__main__':
    unittest.main(verbosity=2)
