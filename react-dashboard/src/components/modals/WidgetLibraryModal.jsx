import { useDispatch, useSelector } from "react-redux";
import { useState } from "react";
import { addWidgetToPage, selectPages } from "../../store/pagesSlice";
import WidgetExistsModal from "./WidgetExistsModal";
import { selectWidgets } from "../../store/widgetsSlice";

export default function WidgetLibraryModal({ isOpen, onClose }) {
  const dispatch = useDispatch();
  const widgets = useSelector(selectWidgets); // all saved widgets
  const widgetArray = Object.values(widgets.widgets); // convert from object to array
  const { pages, currentPage } = useSelector(selectPages);
  const [showDuplicateModal, setShowDuplicateModal] = useState(false);

  if (!isOpen) return null;

  const handleAddWidget = (widgetId) => {
    const currentPageObj = pages.find((p) => p.id === currentPage);
    if (!currentPageObj) return;

    // Check if widget already exists on this page
    if (currentPageObj.widgets && currentPageObj.widgets[widgetId]) {
      setShowDuplicateModal(true);
      return;
    }

    dispatch(
      addWidgetToPage({
        pageId: currentPage,
        widgetId,
      }),
    );
    onClose();
  };

  return (
    <div className="fixed inset-0 bg-black/50 flex items-center justify-center z-50">
      <div className="bg-white rounded-lg shadow-xl w-full max-w-2xl p-6 max-h-[80vh] overflow-y-auto">
        <h3 className="text-xl font-bold mb-4">Select Widget from Library</h3>

        {widgetArray.length === 0 ? (
          <div className="text-center py-8">
            <p className="text-gray-500 mb-4">No widgets in library yet.</p>
            <p className="text-sm text-gray-400">
              Create a widget and it will appear here automatically.
            </p>
          </div>
        ) : (
          <div className="space-y-3">
            {widgetArray.map((widget) => (
              <div
                key={widget.id}
                className="flex items-center justify-between p-4 border border-gray-300 rounded-lg hover:bg-gray-50 cursor-pointer transition-colors"
                onClick={() => handleAddWidget(widget.id)}
              >
                <div className="flex-1">
                  <div className="font-medium text-lg">{widget.name}</div>
                  <div className="text-sm text-gray-600 mt-1">
                    <span className="inline-block px-2 py-1 bg-gray-200 rounded text-xs mr-2">
                      {widget.type}
                    </span>
                    Update: {widget.updateInterval}ms
                    {widget.endpoint && ` | ${widget.endpoint}`}
                  </div>
                </div>
                <button
                  className="px-4 py-2 bg-blue-500 text-white rounded hover:bg-blue-600 transition-colors"
                  onClick={(e) => {
                    e.stopPropagation();
                    handleAddWidget(widget.id);
                  }}
                >
                  Add
                </button>
              </div>
            ))}
          </div>
        )}

        <div className="mt-6">
          <button
            onClick={onClose}
            className="w-full px-4 py-2 border border-gray-300 rounded-lg hover:bg-gray-50 transition-colors"
          >
            Close
          </button>
        </div>
        <WidgetExistsModal
          isOpen={showDuplicateModal}
          onClose={() => setShowDuplicateModal(false)}
        ></WidgetExistsModal>
      </div>
    </div>
  );
}
