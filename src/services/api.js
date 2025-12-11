// Dummy REST API service
const API = {
  /**
   * GET request to fetch data
   * @param {string} endpoint - API endpoint
   * @returns {Promise<object>} Response data
   */
  async get(endpoint) {
    // Simulate network delay
    await new Promise((resolve) => setTimeout(resolve, 300));

    // Return mock data based on endpoint
    if (endpoint.includes("/number")) {
      return { value: Math.floor(Math.random() * 100) };
    }

    if (endpoint.includes("/state")) {
      return { value: Math.random() > 0.5 };
    }

    if (endpoint.includes("/history")) {
      return {
        values: Array.from({ length: 20 }, () =>
          Math.floor(Math.random() * 100)
        ),
      };
    }

    // Default response
    return { value: 0 };
  },

  /**
   * PUT request to update data
   * @param {string} endpoint - API endpoint
   * @param {object} data - Data to send
   * @returns {Promise<object>} Response data
   */
  async put(endpoint, data) {
    // Simulate network delay
    await new Promise((resolve) => setTimeout(resolve, 300));

    console.log("PUT", endpoint, data);
    return { success: true };
  },

  /**
   * POST request to create data
   * @param {string} endpoint - API endpoint
   * @param {object} data - Data to send
   * @returns {Promise<object>} Response data
   */
  async post(endpoint, data) {
    // Simulate network delay
    await new Promise((resolve) => setTimeout(resolve, 300));

    console.log("POST", endpoint, data);
    return { success: true };
  },

  /**
   * DELETE request to remove data
   * @param {string} endpoint - API endpoint
   * @returns {Promise<object>} Response data
   */
  async delete(endpoint) {
    // Simulate network delay
    await new Promise((resolve) => setTimeout(resolve, 300));

    console.log("DELETE", endpoint);
    return { success: true };
  },
};

export default API;
