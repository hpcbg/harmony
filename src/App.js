import React, { useState } from 'react';
import { DashboardProvider, useDashboard } from './context/DashboardContext';
import { useWidgetData } from './hooks';
import DashboardLayout from './components/layout/DashboardLayout';
import DashboardPage from './components/pages/DashboardPage';
import SettingsPage from './components/pages/SettingsPage';
import PageFormModal from './components/modals/PageFormModal';
import WidgetFormModal from './components/modals/WidgetFormModal';
import WidgetLibraryModal from './components/modals/WidgetLibraryModal';
import {
  NumberWidget,
  StateWidget,
  HistoryWidget,
  ButtonWidget
} from './components/widgets';
import API from './services/api';

const DashboardContent = () => {
  const {
    currentPage,
    isEditMode,
    pages,
    widgets,
    widgetLibrary,
    setCurrentPage,
    setIsEditMode,
    addPage,
    updatePage,
    reorderPages,
    deletePage,
    addWidget,
    updateWidget,
    deleteWidget,
    removeWidgetFromPage,
    moveWidget,
    addFromLibrary,
    exportConfig,
    importConfig
  } = useDashboard();

  // Fetch widget data
  const widgetData = useWidgetData(widgets);

  // Modal states
  const [showPageForm, setShowPageForm] = useState(false);
  const [editingPage, setEditingPage] = useState(null);
  
  const [showWidgetForm, setShowWidgetForm] = useState(false);
  const [editingWidget, setEditingWidget] = useState(null);
  //const [isEditingLibraryWidget, setIsEditingLibraryWidget] = useState(false);
  
  const [showLibrary, setShowLibrary] = useState(false);

  // Page handlers
  const handleAddPage = () => {
    setEditingPage(null);
    setShowPageForm(true);
  };

  const handleEditPage = (page) => {
    setEditingPage(page);
    setShowPageForm(true);
  };

  const handlePageFormSubmit = (name) => {
    if (editingPage) {
      updatePage(editingPage.id, { name });
    } else {
      addPage(name);
    }
    setShowPageForm(false);
    setEditingPage(null);
  };

  // Widget handlers
  const handleAddWidget = () => {
    setEditingWidget(null);
    //setIsEditingLibraryWidget(false);
    setShowWidgetForm(true);
  };

  const handleEditWidget = (widget, isLibrary = true) => {
    setEditingWidget(widget);
    setShowWidgetForm(true);
  };

  const handleWidgetFormSubmit = (widgetData) => {
    if (editingWidget) {
      if (editingWidget.libraryId) {
        updateWidget(editingWidget.libraryId, widgetData);
      } else {
        updateWidget(editingWidget.id, widgetData);
      }
    } else {
      addWidget(widgetData);
    }
    setShowWidgetForm(false);
    setEditingWidget(null);
  };

  const handleSelectFromLibrary = (libraryWidget) => {
    addFromLibrary(libraryWidget);
    setShowLibrary(false);
  };

  // Button action handler
  const handleButtonAction = async (widget) => {
    await API.put(widget.endpoint, { action: widget.name });
  };

  // Render widget based on type
  const renderWidget = (widget, data) => {
    switch (widget.type) {
      case 'number':
        return <NumberWidget widget={widget} data={data} />;
      case 'state':
        return <StateWidget widget={widget} data={data} />;
      case 'history':
        return <HistoryWidget widget={widget} data={data} />;
      case 'button':
        return <ButtonWidget widget={widget} onAction={handleButtonAction} />;
      default:
        return <div>Unknown widget type</div>;
    }
  };

  const currentPageData = pages.find(p => p.id === currentPage);

  return (
    <>
      <DashboardLayout
        pages={pages}
        currentPage={currentPage}
        isEditMode={isEditMode}
        onPageChange={setCurrentPage}
        onAddPage={handleAddPage}
        onReorderPages={reorderPages}
        onAddWidget={handleAddWidget}
        onShowLibrary={() => setShowLibrary(true)}
      >
        {currentPage === 'settings' ? (
          <SettingsPage
            isEditMode={isEditMode}
            onToggleEditMode={() => setIsEditMode(!isEditMode)}
            pages={pages}
            widgets={widgets}
            widgetLibrary={widgetLibrary}
            onEditPage={handleEditPage}
            onDeletePage={deletePage}
            onEditWidget={handleEditWidget}
            onDeleteWidget={deleteWidget}
            onExportConfig={exportConfig}
            onImportConfig={importConfig}
          />
        ) : (
          <DashboardPage
            pageData={currentPageData}
            widgets={widgets}
            widgetData={widgetData}
            isEditMode={isEditMode}
            onMoveWidget={moveWidget}
            onRemoveWidget={removeWidgetFromPage}
            onEditWidget={handleEditWidget}
            renderWidget={renderWidget}
          />
        )}
      </DashboardLayout>

      {/* Modals */}
      <PageFormModal
        isOpen={showPageForm}
        onClose={() => {
          setShowPageForm(false);
          setEditingPage(null);
        }}
        onSubmit={handlePageFormSubmit}
        editingPage={editingPage}
      />

      <WidgetFormModal
        isOpen={showWidgetForm}
        onClose={() => {
          setShowWidgetForm(false);
          setEditingWidget(null);
        }}
        onSubmit={handleWidgetFormSubmit}
        editingWidget={editingWidget}
      />

      <WidgetLibraryModal
        isOpen={showLibrary}
        onClose={() => setShowLibrary(false)}
        widgetLibrary={widgetLibrary}
        onSelectWidget={handleSelectFromLibrary}
      />
    </>
  );
};

function App() {
  return (
    <DashboardProvider>
      <DashboardContent />
    </DashboardProvider>
  );
}

export default App;
