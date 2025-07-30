"""
Test suite for AssetX
"""

import pytest
import tempfile
from pathlib import Path
import yaml

from assetx.core.asset import Asset, PhysicsProperties
from assetx.core.converter import FormatConverter
from assetx.core.validator import PhysicsValidator
from assetx.mesh.processor import MeshProcessor
from assetx.meta.manager import MetaManager


class TestAsset:
    """Test Asset class"""
    
    def test_format_detection(self):
        """Test format detection from file extension"""
        with tempfile.NamedTemporaryFile(suffix='.urdf') as f:
            asset = Asset(f.name)
            assert asset.format == 'urdf'
        
        with tempfile.NamedTemporaryFile(suffix='.xml') as f:
            asset = Asset(f.name)
            assert asset.format == 'mjcf'
    
    def test_physics_properties(self):
        """Test PhysicsProperties dataclass"""
        props = PhysicsProperties(
            mass=1.5,
            inertia=[0.1, 0.2, 0.3, 0.0, 0.0, 0.0],
            center_of_mass=[0.0, 0.0, 0.1]
        )
        assert props.mass == 1.5
        assert len(props.inertia) == 6
        assert len(props.center_of_mass) == 3


class TestFormatConverter:
    """Test FormatConverter class"""
    
    def test_supported_conversions(self):
        """Test supported conversion mappings"""
        converter = FormatConverter()
        
        assert converter.can_convert('urdf', 'mjcf')
        assert converter.can_convert('urdf', 'usd') 
        assert not converter.can_convert('genesis', 'urdf')
    
    def test_extension_mapping(self):
        """Test file extension mapping"""
        converter = FormatConverter()
        
        assert converter._get_extension('urdf') == '.urdf'
        assert converter._get_extension('mjcf') == '.xml'
        assert converter._get_extension('usd') == '.usd'
        assert converter._get_extension('genesis') == '.json'


class TestPhysicsValidator:
    """Test PhysicsValidator class"""
    
    def test_triangle_inequality(self):
        """Test inertia triangle inequality check"""
        validator = PhysicsValidator()
        
        # Valid inertia values
        assert validator._check_inertia_triangle_inequality(1.0, 1.0, 1.0)
        assert validator._check_inertia_triangle_inequality(1.0, 2.0, 2.5)
        
        # Invalid inertia values
        assert not validator._check_inertia_triangle_inequality(1.0, 1.0, 3.0)
    
    def test_validation_result(self):
        """Test ValidationResult class"""
        from assetx.core.validator import ValidationResult
        
        result = ValidationResult()
        assert result.is_valid
        
        result.add_warning("Test warning")
        assert len(result.warnings) == 1
        assert result.is_valid  # Warnings don't affect validity
        
        result.add_error("Test error")
        assert len(result.errors) == 1
        assert not result.is_valid  # Errors make it invalid


class TestMeshProcessor:
    """Test MeshProcessor class"""
    
    def test_supported_formats(self):
        """Test supported mesh formats"""
        processor = MeshProcessor()
        
        expected_formats = ['.obj', '.stl', '.ply', '.dae', '.glb', '.gltf']
        for fmt in expected_formats:
            assert fmt in processor.supported_formats
    
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
            assert "test" in asset_meta['tags']
            assert "demo" in asset_meta['tags']
    
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
            assert loaded_meta['test_field'] == 'test_value'
            assert loaded_meta['schema_version'] == "1.0"


# Integration tests
class TestIntegration:
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
    pytest.main([__file__])
