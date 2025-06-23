"""Test validation functions for BARNS system testing."""

import asyncio

from . import BaseValidation


class ValidationTest1(BaseValidation):
    """Basic test validation function."""
    
    @property
    def function_name(self) -> str:
        return "validate_test1"
    
    async def validate(self, params: dict) -> dict:
        """Test function 1 for validation service."""
        await asyncio.sleep(5)
        return {
            "passed": True,
            "details": "validate_test1 passed successfully",
            "data": {
                "test_name": "validate_test1",
                "params_received": params,
                "service": "validation",
                "test_type": "integration"
            }
        }


class ValidationTest2(BaseValidation):
    """Secondary test validation function."""
    
    @property
    def function_name(self) -> str:
        return "validate_test2"
    
    async def validate(self, params: dict) -> dict:
        await asyncio.sleep(5)
        """Test function 2 for validation service."""
        return {
            "passed": True,
            "details": "validate_test2 passed successfully", 
            "data": {
                "test_name": "validate_test2",
                "params_received": params,
                "service": "validation",
                "test_type": "comprehensive"
            }
        } 