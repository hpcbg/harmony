import React from 'react';

const WidgetLibraryModal = ({ 
  isOpen, 
  onClose, 
  widgetLibrary, 
  onSelectWidget 
}) => {
  if (!isOpen) return null;
  
  return (
    <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center 
                    justify-center z-50">
      <div className="bg-white rounded-lg shadow-xl w-full max-w-2xl p-6 
                      max-h-[80vh] overflow-y-auto">
        <h3 className="text-xl font-bold mb-4">Select Widget from Library</h3>
        
        {widgetLibrary.length === 0 ? (
          <div className="text-center py-8">
            <p className="text-gray-500 mb-4">No widgets in library yet.</p>
            <p className="text-sm text-gray-400">
              Create a widget and click "Save to Library" to add it here.
            </p>
          </div>
        ) : (
          <div className="space-y-3">
            {widgetLibrary.map(widget => (
              <div
                key={widget.id}
                className="flex items-center justify-between p-4 border rounded-lg 
                         hover:bg-gray-50 cursor-pointer transition-colors"
                onClick={() => onSelectWidget(widget)}
              >
              <div className="flex-1">
                <div className="font-medium text-lg">{widget.name}</div>
                <div className="text-sm text-gray-600 mt-1">
                  <span className="inline-block px-2 py-1 bg-gray-200 rounded 
                                  text-xs mr-2">
                    {widget.type}
                  </span>
                  Update: {widget.updateInterval}ms
                  {widget.endpoint && ` | ${widget.endpoint}`}
                </div>
              </div>
              <button
                className="px-4 py-2 bg-blue-500 text-white rounded 
                          hover:bg-blue-600 transition-colors"
                onClick={(e) => {
                  e.stopPropagation();
                  onSelectWidget(widget);
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
            className="w-full px-4 py-2 border rounded-lg hover:bg-gray-50 
                     transition-colors"
          >
            Close
          </button>
        </div>
      </div>
    </div>
  );
};

export default WidgetLibraryModal;