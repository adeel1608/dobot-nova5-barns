"""Validation functions module for BARNS validation system."""

import importlib
import inspect
import os
from typing import Dict, Callable
from abc import ABC, abstractmethod


class BaseValidation(ABC):
    """Base class for validation functions."""
    
    @abstractmethod
    def validate(self, params: dict) -> dict:
        """Perform validation with given parameters."""
        pass
    
    @property
    @abstractmethod
    def function_name(self) -> str:
        """Return the function name for registration."""
        pass


def load_validation_functions() -> Dict[str, Callable]:
    """Dynamically load all validation functions from modules in this package."""
    validators = {}
    current_dir = os.path.dirname(__file__)
    
    for filename in os.listdir(current_dir):
        if filename.endswith('.py') and filename != '__init__.py':
            module_name = filename[:-3]
            
            try:
                module = importlib.import_module(f'.{module_name}', package=__name__)
                
                # Load validation classes
                for name, obj in inspect.getmembers(module):
                    if (inspect.isclass(obj) and 
                        issubclass(obj, BaseValidation) and 
                        obj != BaseValidation):
                        
                        validator_instance = obj()
                        validators[validator_instance.function_name] = validator_instance.validate
                
                # Load standalone functions
                for name, obj in inspect.getmembers(module):
                    if (inspect.isfunction(obj) and 
                        name.startswith('validate_') and
                        not name.startswith('validate_test')):
                        
                        validators[name] = obj
                        
            except Exception as e:
                print(f"Warning: Could not load validation module {module_name}: {e}")
    
    return validators


def create_simple_validator(function_name: str, validation_func: Callable) -> Dict[str, Callable]:
    """
    Helper function to create a simple validator from a function
    
    Args:
        function_name: Name to register the function under
        validation_func: The validation function
        
    Returns:
        Dictionary with the validator
    """
    return {function_name: validation_func} 