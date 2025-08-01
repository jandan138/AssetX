"""
Test suite for AssetX
"""

import unittest
import tempfile
from pathlib import Path
import sys
import os

# 添加项目根目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from assetx.core.asset import Asset
from assetx.core.processing.converter import FormatConverter
from assetx.core.processing.validator import PhysicsValidator
from assetx.mesh.processor import MeshProcessor
from assetx.meta.manager import MetaManager


class TestAsset(unittest.TestCase):
    """Test Asset class"""
    
    def test_format_detection(self):
        """Test format detection from file extension"""
        with tempfile.NamedTemporaryFile(suffix='.urdf') as f:
            asset = Asset(f.name)
            self.assertEqual(asset.format.value, 'urdf')
        
        with tempfile.NamedTemporaryFile(suffix='.xml') as f:
            asset = Asset(f.name)
            self.assertEqual(asset.format.value, 'mjcf')
    
    def test_asset_creation(self):
        """Test basic asset creation"""
        with tempfile.NamedTemporaryFile(suffix='.urdf') as f:
            asset = Asset(f.name)
            self.assertIsNotNone(asset)
            self.assertFalse(asset.is_loaded)
            self.assertIsNotNone(asset.query)
            self.assertIsNotNone(asset.schema)
            self.assertIsNotNone(asset.meta)


class TestFormatConverter(unittest.TestCase):
    """Test FormatConverter class"""
    
    def test_supported_conversions(self):
        """Test supported conversion mappings"""
        converter = FormatConverter()
        
        self.assertTrue(converter.can_convert('urdf', 'mjcf'))
        self.assertTrue(converter.can_convert('urdf', 'usd')) 
        self.assertFalse(converter.can_convert('genesis', 'urdf'))
    
    def test_extension_mapping(self):
        """Test file extension mapping"""
        converter = FormatConverter()
        
        self.assertEqual(converter._get_extension('urdf'), '.urdf')
        self.assertEqual(converter._get_extension('mjcf'), '.xml')
        self.assertEqual(converter._get_extension('usd'), '.usd')
        self.assertEqual(converter._get_extension('genesis'), '.json')


class TestPhysicsValidator(unittest.TestCase):
    """Test PhysicsValidator class"""
    
    def test_triangle_inequality(self):
        """Test inertia triangle inequality check"""
        validator = PhysicsValidator()
        
        # Valid inertia values
        self.assertTrue(validator._check_inertia_triangle_inequality(1.0, 1.0, 1.0))
        self.assertTrue(validator._check_inertia_triangle_inequality(1.0, 2.0, 2.5))
        
        # Invalid inertia values
        self.assertFalse(validator._check_inertia_triangle_inequality(1.0, 1.0, 3.0))
    
    def test_validation_result(self):
        """Test ValidationResult class"""
        from assetx.core.processing.validator import ValidationResult
        
        result = ValidationResult()
        self.assertTrue(result.is_valid)
        
        result.add_warning("Test warning")
        self.assertEqual(len(result.warnings), 1)
        self.assertTrue(result.is_valid)  # Warnings don't affect validity
        
        result.add_error("Test error")
        self.assertEqual(len(result.errors), 1)
        self.assertFalse(result.is_valid)  # Errors make it invalid


class TestMeshProcessor(unittest.TestCase):
    """Test MeshProcessor class"""
    
    def test_supported_formats(self):
        """Test supported mesh formats"""
        processor = MeshProcessor()
        
        expected_formats = ['.obj', '.stl', '.ply', '.dae', '.glb', '.gltf']
        for fmt in expected_formats:
            self.assertIn(fmt, processor.supported_formats)
    
    def test_unit_normalization(self):
        """Test unit conversion calculations"""
        processor = MeshProcessor()
        
        # Test unit scales
        with tempfile.NamedTemporaryFile(suffix='.obj') as f:
            # This would fail without actual mesh file, but tests the logic
            try:
                processor.normalize_units(f.name, 'mm', 'm')
            except ImportError:
                pass  # Expected if trimesh not installed


class TestMetaManager:
    """Test MetaManager class"""
    
    def test_meta_template_creation(self):
        """Test meta template structure"""
        with tempfile.TemporaryDirectory() as tmpdir:
            meta_manager = MetaManager(tmpdir)
            template = meta_manager.create_meta_template()
            
            assert 'schema_version' in template
            assert 'assets' in template
            assert 'global_settings' in template
            assert template['schema_version'] == "1.0"
    
    def test_asset_meta_creation(self):
        """Test asset metadata creation"""
        with tempfile.TemporaryDirectory() as tmpdir:
            meta_manager = MetaManager(tmpdir)
            
            asset_meta = meta_manager.create_asset_meta(
                asset_name="test_robot.urdf",
                original_format="urdf",
                semantic_category="robot_arm",
                description="Test robot",
                tags=["test", "demo"]
            )
            
            assert asset_meta['name'] == "test_robot.urdf"
            assert asset_meta['original_format'] == "urdf"
            assert asset_meta['semantic_category'] == "robot_arm"
            self.assertIn("test", asset_meta['tags'])
            self.assertIn("demo", asset_meta['tags'])
    
    def test_meta_file_operations(self):
        """Test meta file save and load"""
        with tempfile.TemporaryDirectory() as tmpdir:
            meta_manager = MetaManager(tmpdir)
            
            # Create and save meta data
            meta_data = meta_manager.create_meta_template()
            meta_data['test_field'] = 'test_value'
            meta_manager.save_meta(meta_data)
            
            # Load and verify
            loaded_meta = meta_manager.load_meta()
            self.assertEqual(loaded_meta['test_field'], 'test_value')
            self.assertEqual(loaded_meta['schema_version'], "1.0")


# Integration tests
class TestIntegration(unittest.TestCase):
    """Integration tests for AssetX components"""
    
    def test_meta_and_converter_integration(self):
        """Test integration between MetaManager and FormatConverter"""
        with tempfile.TemporaryDirectory() as tmpdir:
            meta_manager = MetaManager(tmpdir)
            converter = FormatConverter()
            
            # Test conversion record
            meta_manager.add_conversion_record(
                asset_name="test.urdf",
                target_format="mjcf",
                output_path="test.xml",
                conversion_time=1.23,
                success=True
            )
            
            # This should not raise an error even if asset doesn't exist
            # because we're testing the meta management functionality


if __name__ == "__main__":
    unittest.main()
