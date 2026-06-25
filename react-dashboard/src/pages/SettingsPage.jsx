import { useDispatch, useSelector } from "react-redux";
import { Edit, Trash2 } from "lucide-react";
import {
  toggleEditMode,
  deletePage,
  startEditPage,
  setPagesFromConfig,
  selectPages,
} from "../store/pagesSlice";
import {
  startEditWidget,
  setWidgetsFromConfig,
  selectWidgets,
} from "../store/widgetsSlice";
import { deleteWidgetAndRemoveFromPages } from "../store/widgetsSlice";
import useConfirm from "../hooks/useConfirm";
import ConfirmModal from "../components/modals/ConfirmModal";

export default function SettingsPage() {
  const { confirm, openConfirm, closeConfirm, isOpen } = useConfirm();
  const dispatch = useDispatch();
  const { pages, editMode } = useSelector(selectPages);
  const widgets = useSelector(selectWidgets);
  const widgetArray = Object.values(widgets.widgets);
  const handleToggleEditMode = () => dispatch(toggleEditMode());
  const handleDeletePageClick = (pageId) => {
    openConfirm({
      type: "delete-page",
      pageId,
    });
  };
  const handleDeleteWidgetClick = (widgetId) => {
    openConfirm({
      type: "delete-widget",
      widgetId,
    });
  };
  const handleEditWidget = (widgetId) => dispatch(startEditWidget(widgetId));
  const handleConfirm = () => {
    if (confirm.type === "delete-page") {
      dispatch(deletePage(confirm.pageId));
    }

    if (confirm.type === "delete-widget") {
      dispatch(deleteWidgetAndRemoveFromPages(confirm.widgetId));
    }

    closeConfirm();
  };

  const importConfig = () => {
    const input = document.createElement("input");
    input.type = "file";
    input.accept = "application/json";

    input.onchange = (e) => {
      const file = e.target.files?.[0];
      if (!file) return;

      const reader = new FileReader();

      reader.onload = () => {
        try {
          const config = JSON.parse(reader.result);
          if (
            !config ||
            !Array.isArray(config.pages) ||
            !Array.isArray(config.widgets)
          ) {
            throw new Error("Invalid config shape");
          }

          // ---- widgets ----
          const widgetsById = {};
          config.widgets.forEach((w) => {
            if (!w?.id) return;
            widgetsById[w.id] = w;
          });

          // ---- pages ----
          const pages = config.pages.map((p) => {
            const widgets = {};

            if (Array.isArray(p.widgets)) {
              p.widgets.forEach((w) => {
                if (!w?.id) return;

                widgets[w.id] = {
                  x: w.x ?? 0,
                  y: w.y ?? 0,
                  width: w.width ?? 300,
                  height: w.height ?? 200,
                };
              });
            }

            return {
              id: p.id,
              name: p.name,
              widgets,
            };
          });

          dispatch(setWidgetsFromConfig(widgetsById));
          dispatch(setPagesFromConfig(pages));

          alert("Configuration imported successfully!");
        } catch (err) {
          console.error("Import error:", err);
          alert("Invalid configuration file");
        }
      };

      reader.readAsText(file);
    };

    input.click();
  };

  const exportConfig = async () => {
    const config = {
      pages: pages
        .filter((p) => p.id !== "settings")
        .map((p) => ({
          id: p.id,
          name: p.name,
          widgets: Object.entries(p.widgets || {}).map(([widgetId, pos]) => ({
            id: widgetId,
            ...pos,
          })),
        })),
      widgets: widgetArray,
    };

    const blob = new Blob([JSON.stringify(config, null, 2)], {
      type: "application/json",
    });

    if ("showSaveFilePicker" in window) {
      try {
        const handle = await window.showSaveFilePicker({
          suggestedName: "dashboard-config.json",
          types: [
            {
              description: "JSON Files",
              accept: { "application/json": [".json"] },
            },
          ],
        });

        const writable = await handle.createWritable();
        await writable.write(blob);
        await writable.close();
      } catch (err) {
        // User cancelled the dialog or error occurred
        if (err.name !== "AbortError") {
          console.error("Export error:", err);
          alert("Failed to export configuration");
        }
      }
    } else {
      // Fallback for browsers that don't support File System Access API
      const url = URL.createObjectURL(blob);
      const a = document.createElement("a");
      a.href = url;
      a.download = "dashboard-config.json";
      a.click();
      URL.revokeObjectURL(url);
    }
  };

  return (
    <div className="p-6 overflow-y-auto h-full">
      <div className="max-w-4xl">
        {/* Edit Mode Toggle */}
        <div className="mb-6">
          <button
            onClick={handleToggleEditMode}
            className={`flex items-center gap-2 px-6 py-3 rounded-lg font-semibold transition-colors ${
              editMode
                ? "bg-orange-500 text-white hover:bg-orange-600"
                : "bg-blue-500 text-white hover:bg-blue-600"
            }`}
          >
            <Edit size={20} />
            {editMode ? "Exit Edit Mode" : "Enter Edit Mode"}
          </button>
          {editMode && (
            <p className="text-sm text-gray-600 mt-2">
              Edit mode is active. You can now add/edit/delete pages and
              widgets, and move widgets on pages.
            </p>
          )}
        </div>

        {/* All Pages */}
        <div className="bg-white rounded-lg shadow p-6 mb-4">
          <h3 className="text-lg font-semibold mb-3">All Pages</h3>
          {pages.length === 0 ? (
            <p className="text-gray-500">No pages created yet</p>
          ) : (
            <div className="space-y-2">
              {pages
                .filter((page) => page.id !== "settings")
                .map((page) => (
                  <div
                    key={page.id}
                    className="flex items-center justify-between p-3 bg-gray-50 rounded"
                  >
                    <div>
                      <div className="font-medium">{page.name}</div>
                      <div className="text-sm text-gray-600">
                        {Object.keys(page.widgets).length} widget(s)
                      </div>
                    </div>
                    {editMode && (
                      <div className="flex gap-2">
                        <button
                          onClick={() => dispatch(startEditPage(page))}
                          className="p-2 text-blue-500 hover:text-blue-700 transition-colors"
                        >
                          <Edit size={18} />
                        </button>
                        <button
                          onClick={() => handleDeletePageClick(page.id)}
                          className="p-2 text-red-500 hover:text-red-700 transition-colors"
                        >
                          <Trash2 size={18} />
                        </button>
                      </div>
                    )}
                  </div>
                ))}
            </div>
          )}
        </div>

        {/* All Widgets */}
        <div className="bg-white rounded-lg shadow p-6 mb-4">
          <h3 className="text-lg font-semibold mb-3">Reusable Widgets</h3>
          {widgetArray.length === 0 ? (
            <p className="text-gray-500">No widgets created yet</p>
          ) : (
            <div className="space-y-4">
              {widgetArray.map((widget) => (
                <div
                  key={widget.id}
                  className="flex items-center justify-between p-3 bg-purple-50 rounded"
                >
                  <div>
                    <div className="font-medium">{widget.name}</div>
                    <div className="text-sm text-gray-600">
                      Type: {widget.type} | Update: {widget.updateInterval}ms
                      {widget.endpoint && ` | Endpoint: ${widget.endpoint}`}
                    </div>
                  </div>
                  {editMode && (
                    <div className="flex gap-2">
                      <button
                        onClick={() => handleEditWidget(widget.id)}
                        className="p-2 text-blue-500 hover:text-blue-700 transition-colors"
                      >
                        <Edit size={18} />
                      </button>
                      <button
                        onClick={() => handleDeleteWidgetClick(widget.id)}
                        className="p-2 text-red-500 hover:text-red-700 transition-colors"
                      >
                        <Trash2 size={18} />
                      </button>
                    </div>
                  )}
                </div>
              ))}
            </div>
          )}
        </div>

        {/* Widget Types */}
        <div className="bg-white rounded-lg shadow p-6 mb-4">
          <h3 className="text-lg font-semibold mb-3">Widget Types Available</h3>
          <div className="space-y-3">
            <div className="p-3 bg-blue-50 rounded">
              <div className="font-medium text-blue-800">Value</div>
              <div className="text-sm text-gray-600">
                Displays a value from the API
              </div>
            </div>
            <div className="p-3 bg-green-50 rounded">
              <div className="font-medium text-green-800">State (Boolean)</div>
              <div className="text-sm text-gray-600">
                Shows active/inactive status with visual indicator
              </div>
            </div>
            <div className="p-3 bg-purple-50 rounded">
              <div className="font-medium text-purple-800">
                History (Line chart)
              </div>
              <div className="text-sm text-gray-600">
                Visualizes historical data as line chart
              </div>
            </div>
            <div className="p-3 bg-purple-50 rounded">
              <div className="font-medium text-purple-800">
                History (Bar chart)
              </div>
              <div className="text-sm text-gray-600">
                Visualizes historical data as a bar chart
              </div>
            </div>
            <div className="p-3 bg-orange-50 rounded">
              <div className="font-medium text-orange-800">Button (Action)</div>
              <div className="text-sm text-gray-600">
                Sends PUT requests to trigger actions
              </div>
            </div>
            <div className="p-3 bg-red-50 rounded">
              <div className="font-medium text-red-800">Image</div>
              <div className="text-sm text-gray-600">
                Visualizes image from the API
              </div>
            </div>
          </div>
        </div>

        {/* Export/Import */}
        <div className="bg-white rounded-lg shadow p-6 mb-4">
          <h3 className="text-lg font-semibold mb-3">
            Export/Import Configuration
          </h3>
          <div className="flex space-x-2">
            <button
              onClick={importConfig}
              className="flex-1 px-4 py-2 bg-blue-500 text-white rounded hover:bg-blue-600 transition-colors"
            >
              Import Configuration
            </button>
            <button
              onClick={exportConfig}
              className="flex-1 px-4 py-2 bg-green-500 text-white rounded hover:bg-green-600 transition-colors"
            >
              Export Configuration
            </button>
          </div>
        </div>
        <ConfirmModal
          isOpen={isOpen}
          message={
            confirm?.type === "delete-page"
              ? "Are you sure you want to delete this page?"
              : "Are you sure you want to delete this widget?"
          }
          onConfirm={handleConfirm}
          onCancel={closeConfirm}
        />
      </div>
    </div>
  );
}
