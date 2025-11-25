import React from 'react';
import { Edit, Trash2 } from 'lucide-react';

const SettingsPage = ({ 
  isEditMode,
  onToggleEditMode,
  pages,
  widgets,
  widgetLibrary,
  onEditPage,
  onDeletePage,
  onEditWidget,
  onDeleteWidget,
  //onDeleteLibraryWidget,
  onExportConfig,
  onImportConfig
}) => {
  return (
    <div className="p-6 overflow-y-auto h-full">
      <div className="max-w-4xl">
        {/* Edit Mode Toggle */}
        <div className="mb-6">
          <button
            onClick={onToggleEditMode}
            className={`flex items-center gap-2 px-6 py-3 rounded-lg font-semibold 
                       transition-colors ${
              isEditMode 
                ? 'bg-orange-500 text-white hover:bg-orange-600' 
                : 'bg-blue-500 text-white hover:bg-blue-600'
            }`}
          >
            <Edit size={20} />
            {isEditMode ? 'Exit Edit Mode' : 'Enter Edit Mode'}
          </button>
          {isEditMode && (
            <p className="text-sm text-gray-600 mt-2">
              Edit mode is active. You can now add/edit/delete pages and widgets, 
              and move widgets on pages.
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
              {pages.map(page => (
                <div key={page.id} 
                     className="flex items-center justify-between p-3 bg-gray-50 rounded">
                  <div>
                    <div className="font-medium">{page.name}</div>
                    <div className="text-sm text-gray-600">
                      {page.widgets.length} widget(s)
                    </div>
                  </div>
                  {isEditMode && (
                    <div className="flex gap-2">
                      <button
                        onClick={() => onEditPage(page)}
                        className="p-2 text-blue-500 hover:text-blue-700 transition-colors"
                      >
                        <Edit size={18} />
                      </button>
                      <button
                        onClick={() => onDeletePage(page.id)}
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
          
          {widgetLibrary.length === 0 ? (
            <p className="text-gray-500">No widgets created yet</p>
          ) : (
            <div className="space-y-4">
              {/* Library Widgets */}
              {widgetLibrary.length > 0 && (
                <div>
                  <div className="space-y-2">
                    {widgetLibrary.map(widget => (
                      <div key={widget.id} 
                           className="flex items-center justify-between p-3 bg-purple-50 rounded">
                        <div>
                          <div className="font-medium">{widget.name}</div>
                          <div className="text-sm text-gray-600">
                            Type: {widget.type} | Update: {widget.updateInterval}ms
                            {widget.endpoint && ` | Endpoint: ${widget.endpoint}`}
                          </div>
                        </div>
                        {isEditMode && (
                          <div className="flex gap-2">
                            <button
                              onClick={() => onEditWidget(widget, true)}
                              className="p-2 text-blue-500 hover:text-blue-700 transition-colors"
                            >
                              <Edit size={18} />
                            </button>
                            <button
                              onClick={() => onDeleteWidget(widget.id)}
                              className="p-2 text-red-500 hover:text-red-700 transition-colors"
                            >
                              <Trash2 size={18} />
                            </button>
                          </div>
                        )}
                      </div>
                    ))}
                  </div>
                </div>
              )}
            </div>
          )}
        </div>

        {/* Widget Types */}
        <div className="bg-white rounded-lg shadow p-6 mb-4">
          <h3 className="text-lg font-semibold mb-3">Widget Types Available</h3>
          <div className="space-y-3">
            <div className="p-3 bg-blue-50 rounded">
              <div className="font-medium text-blue-800">Number</div>
              <div className="text-sm text-gray-600">
                Displays a numeric value from the API
              </div>
            </div>
            <div className="p-3 bg-green-50 rounded">
              <div className="font-medium text-green-800">State (Boolean)</div>
              <div className="text-sm text-gray-600">
                Shows active/inactive status with visual indicator
              </div>
            </div>
            <div className="p-3 bg-purple-50 rounded">
              <div className="font-medium text-purple-800">History (Graph)</div>
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
          </div>
        </div>
        
        {/* Export/Import */}
        <div className="bg-white rounded-lg shadow p-6">
          <h3 className="text-lg font-semibold mb-3">
            Export/Import Configuration
          </h3>
          <div className="space-y-2">
            <button
              onClick={onExportConfig}
              className="w-full px-4 py-2 bg-green-500 text-white rounded 
                       hover:bg-green-600 transition-colors"
            >
              Export Configuration
            </button>
            <button
              onClick={onImportConfig}
              className="w-full px-4 py-2 bg-blue-500 text-white rounded 
                       hover:bg-blue-600 transition-colors"
            >
              Import Configuration
            </button>
          </div>
        </div>
      </div>
    </div>
  );
};

export default SettingsPage;