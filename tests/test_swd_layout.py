"""SWD布局纯解析，禁止把数组大小退化为元素大小；无设备访问。"""
from types import SimpleNamespace
import unittest
from test_execution import ROOT
import sys
sys.path.insert(0,str(ROOT/'tools'))
from inspect_mech import layout_type


class Die:
    def __init__(self, tag, attributes=None, target=None, children=()):
        self.tag=tag
        self.attributes={key:SimpleNamespace(value=value) for key,value in (attributes or {}).items()}
        self.target=target;self.children=children
        if target is not None:self.attributes['DW_AT_type']=SimpleNamespace(value=1)
    def get_DIE_from_attribute(self, _):return self.target
    def iter_children(self):return iter(self.children)


class SwdLayoutTests(unittest.TestCase):
    def test_motor_reply_array_and_alias_keep_full_size(self):
        item=Die('DW_TAG_structure_type',{'DW_AT_byte_size':32})
        array=Die('DW_TAG_array_type',target=item,children=[Die('DW_TAG_subrange_type',{'DW_AT_upper_bound':7})])
        alias=Die('DW_TAG_typedef',target=array)
        self.assertEqual(layout_type(alias),(256,True))
    def test_multi_dimension_and_nonzero_lower_bound(self):
        item=Die('DW_TAG_base_type',{'DW_AT_byte_size':8})
        array=Die('DW_TAG_array_type',target=item,children=[Die('DW_TAG_subrange_type',{'DW_AT_count':2}),Die('DW_TAG_subrange_type',{'DW_AT_lower_bound':1,'DW_AT_upper_bound':3})])
        self.assertEqual(layout_type(array),(48,True))
    def test_unknown_array_size_is_rejected(self):
        item=Die('DW_TAG_base_type',{'DW_AT_byte_size':4})
        with self.assertRaises(ValueError):layout_type(Die('DW_TAG_array_type',target=item))
    def test_scalar_alias_is_not_array(self):
        self.assertEqual(layout_type(Die('DW_TAG_typedef',target=Die('DW_TAG_base_type',{'DW_AT_byte_size':8}))),(8,False))
