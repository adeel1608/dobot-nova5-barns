/**
 * Recipes API
 * Fetches available recipes for order creation
 */

import { apiClient } from './base';

export const recipesAPI = {
  // Fetch all available recipes
  fetchRecipes: () => 
    apiClient.getList('/recipes', {}, 'recipes')
      .then(result => ({
        ...result,
        data: result.data || []
      }))
}; 