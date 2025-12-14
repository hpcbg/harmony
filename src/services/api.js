// Dummy REST API service
const API = {
  /**
   * GET request to fetch data
   * @param {string} endpoint - API endpoint
   * @returns {Promise<object>} Response data
   */
  async get(endpoint) {
    // Simulate network delay
    if (endpoint.includes("entities")) {
      const service = "openiot";
      const servicePath = "/";

      // Sample endpoints:
      // http://192.168.1.105:1026/v2/entities/M5Stick:001/attrs/buttonBlue

      const res = await fetch(endpoint, {
        method: "GET",
        headers: {
          "Fiware-Service": service,
          "Fiware-ServicePath": servicePath,
          Accept: "application/json",
        },
      });

      if (!res.ok) {
        console.error("FIWARE error:", await res.text());
        return null;
      }

      let res1 = await res.json();

      if (typeof res1.value === "string" && res1.value.includes("http")) {
        return {
          imageUrl: res1.value,
        };
      }
      return res1;
    } else if (endpoint.includes("camera")) {
      return {
        imageUrl: `${endpoint}?t=${Date.now()}`,
      };
    }

    await new Promise((resolve) => setTimeout(resolve, 300));

    // Return mock data based on endpoint
    if (endpoint.includes("/number")) {
      return { value: Math.floor(Math.random() * 100) };
    }

    if (endpoint.includes("/state")) {
      return { value: Math.random() > 0.5 };
    }

    if (
      endpoint.includes("/historyLineChart") ||
      endpoint.includes("/historyBarChart")
    ) {
      // Return a SINGLE value that will be accumulated over time
      return {
        value: Math.floor(Math.random() * 100),
      };
    }

    if (endpoint.includes("/image")) {
      // Return a random image from unsplash
      const randomId = Math.floor(Math.random() * 1000);
      return {
        imageUrl: `https://picsum.photos/seed/${randomId}/800/600`,
        // Alternative: Use a placeholder service
        // imageUrl: `https://via.placeholder.com/800x600/4A90E2/ffffff?text=Image+${randomId}`
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
