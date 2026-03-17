import { useState, useEffect } from "react";
import { useDispatch, useSelector } from "react-redux";
import {
  updateWidget,
  createWidget,
  stopEditWidget,
  selectWidgets,
} from "../../store/widgetsSlice";
import { addWidgetToPage, selectPages } from "../../store/pagesSlice";

export default function WidgetFormModal() {
  const dispatch = useDispatch();
  const { currentPage } = useSelector(selectPages);
  const { editingWidgetId, creatingNewWidget } = useSelector(selectWidgets);
  const editingWidget = useSelector((state) =>
    editingWidgetId ? state.widgets.widgets[editingWidgetId] : null,
  );
  const [formData, setFormData] = useState({
    name: "",
    type: "number",
    updateInterval: 5000,
    maxValues: 20,
    endpoint: "/api/data/number",
  });

  // Initialize form when modal opens or editingWidget changes
  useEffect(() => {
    if (editingWidget) {
      setFormData({
        name: editingWidget.name,
        type: editingWidget.type,
        updateInterval: editingWidget.updateInterval,
        maxValues: editingWidget.maxValues || 20,
        endpoint: editingWidget.endpoint || `/api/data/${editingWidget.type}`,
      });
    } else if (creatingNewWidget) {
      setFormData({
        name: "",
        type: "number",
        updateInterval: 5000,
        maxValues: 20,
        endpoint: "/api/data/number",
      });
    }
  }, [creatingNewWidget, editingWidget]);

  const handleChange = (field, value) =>
    setFormData((prev) => ({ ...prev, [field]: value }));

  const handleTypeChange = (type) => {
    setFormData((prev) => ({
      ...prev,
      type,
      endpoint: `/api/data/${type}`,
    }));
  };

  const handleClose = () => {
    dispatch(stopEditWidget());
  };

  const handleSubmit = () => {
    if (!formData.name.trim()) return;

    if (editingWidget) {
      dispatch(updateWidget({ id: editingWidget.id, ...formData }));
    } else {
      const action = dispatch(
        createWidget({
          name: formData.name,
          type: formData.type,
          updateInterval: formData.updateInterval,
          endpoint: formData.endpoint,
          maxValues: formData.maxValues,
        }),
      );

      const widgetId = action.payload.id;

      dispatch(
        addWidgetToPage({
          pageId: currentPage,
          widgetId: widgetId,
        }),
      );
    }

    handleClose();
  };

  // Don't render anything if no widget is being edited
  if (!creatingNewWidget && !editingWidget) return null;

  return (
    <div className="fixed inset-0 bg-black/50 flex items-center justify-center z-50">
      <div className="bg-white rounded-lg shadow-xl w-full max-w-md p-6 max-h-[90vh] overflow-y-auto">
        <h3 className="text-xl font-bold mb-4">
          {editingWidget ? "Edit Widget" : "Create New Widget"}
        </h3>

        <div className="space-y-4">
          {/* Name */}
          <div>
            <label className="block text-sm font-medium mb-1">Name</label>
            <input
              type="text"
              value={formData.name}
              onChange={(e) => handleChange("name", e.target.value)}
              className="w-full px-3 py-2 border border-gray-300 rounded-lg focus:outline-none focus:ring-2 focus:ring-blue-500"
              placeholder="Widget name"
            />
          </div>

          {/* Type */}
          <div>
            <label className="block text-sm font-medium mb-1">Type</label>
            <select
              value={formData.type}
              onChange={(e) => handleTypeChange(e.target.value)}
              disabled={!!editingWidget}
              className="w-full px-3 py-2 border border-gray-300 rounded-lg focus:outline-none focus:ring-2 focus:ring-blue-500  disabled:bg-gray-100
                          disabled:text-gray-600
                          disabled:border-gray-200
                          disabled:cursor-not-allowed"
            >
              <option value="number">Value</option>
              <option value="state">State (Boolean)</option>
              <option value="historyLineChart">History (Line chart)</option>
              <option value="historyBarChart">History (Bar chart)</option>
              <option value="button">Button (Action)</option>
              <option value="image">Image</option>
            </select>
          </div>

          {/* Update Interval */}
          <div>
            <label className="block text-sm font-medium mb-1">
              Update Interval (ms)
            </label>
            <input
              type="number"
              value={formData.updateInterval}
              onChange={(e) =>
                handleChange("updateInterval", parseInt(e.target.value))
              }
              className="w-full px-3 py-2 border border-gray-300 rounded-lg focus:outline-none focus:ring-2 focus:ring-blue-500"
              min="1000"
              step="1000"
            />
          </div>

          {/* Max Values */}
          {(formData.type === "historyLineChart" ||
            formData.type === "historyBarChart") && (
            <div>
              <label className="block text-sm font-medium mb-1">
                Max Values
              </label>
              <input
                type="number"
                value={formData.maxValues}
                onChange={(e) =>
                  handleChange("maxValues", parseInt(e.target.value))
                }
                className="w-full px-3 py-2 border border-gray-300 rounded-lg focus:outline-none focus:ring-2 focus:ring-blue-500"
                min="5"
                max="100"
              />
            </div>
          )}

          {/* API Endpoint */}
          <div>
            <label className="block text-sm font-medium mb-1">
              API Endpoint
            </label>
            <input
              type="text"
              value={formData.endpoint}
              onChange={(e) => handleChange("endpoint", e.target.value)}
              className="w-full px-3 py-2 border border-gray-300 rounded-lg focus:outline-none focus:ring-2 focus:ring-blue-500"
              placeholder="/api/data/endpoint"
            />
          </div>
        </div>

        {/* Actions */}
        <div className="flex gap-3 mt-6">
          <button
            onClick={handleSubmit}
            disabled={!formData.name?.trim()}
            className="flex-1 px-4 py-2 bg-blue-500 text-white rounded-lg hover:bg-blue-600 disabled:opacity-50 disabled:cursor-not-allowed transition-colors"
          >
            {editingWidget ? "Update Widget" : "New Widget"}
          </button>
          <button
            onClick={handleClose}
            className="flex-1 px-4 py-2 border border-gray-300 rounded-lg hover:bg-gray-50 transition-colors"
          >
            Cancel
          </button>
        </div>
      </div>
    </div>
  );
}
