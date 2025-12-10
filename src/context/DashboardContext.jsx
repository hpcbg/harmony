import React, { createContext, useContext, useCallback, useState } from 'react';
import { useLocalStorage } from '../hooks';
import { generateId } from '../utils/widgetUtils';
import ConfirmModal from '../components/modals/ConfirmModal';

const DashboardContext = createContext(null);

export const useDashboard = () => {
  const context = useContext(DashboardContext);
  if (!context) {
    throw new Error('useDashboard must be used within DashboardProvider');
  }
  return context;
};

export const DashboardProvider = ({ children }) => {
  const [currentPage, setCurrentPage] = useLocalStorage('dashboard-current-page', 'home');
  const [isEditMode, setIsEditMode] = useLocalStorage('dashboard-edit-mode', false);
  
  const [pages, setPages] = useLocalStorage('dashboard-pages', [
    { id: 'home', name: 'Home', widgets: [] }
  ]);
  
  const [widgets, setWidgets] = useLocalStorage('dashboard-widgets', []);
  const [widgetLibrary, setWidgetLibrary] = useLocalStorage('dashboard-widget-library', []);

  const [confirmModal, setConfirmModal] = useState({
    isOpen: false,
    message: "",
    onConfirm: null
  });

  const openConfirm = useCallback((message, onConfirm) => {
    setConfirmModal({
      isOpen: true,
      message,
      onConfirm
    });
  }, []);

  const closeConfirm = useCallback(() => {
    setConfirmModal(prev => ({ ...prev, isOpen: false }));
  }, []);

  // Page operations
  const addPage = useCallback((name) => {
    const newPage = {
      id: generateId(),
      name,
      widgets: []
    };
    setPages(prev => [...prev, newPage]);
    setCurrentPage(newPage.id);
  }, [setPages, setCurrentPage]);

  const updatePage = useCallback((pageId, updates) => {
    setPages(prev => prev.map(page => 
      page.id === pageId ? { ...page, ...updates } : page
    ));
  }, [setPages]);

  const reorderPages = useCallback((newPages) => {
    setPages(newPages);
  }, [setPages]);

  const deletePage = useCallback((pageId) => {
    openConfirm('Are you sure you want to delete this page?', () => {

      if (pages.length === 1) {
        alert('Cannot delete the last page');
        return;
      }
    
      setPages(prev => prev.filter(p => p.id !== pageId));
    
      if (currentPage === pageId) {
        setCurrentPage(pages[0].id);
      }
    });
  }, [pages, currentPage, setPages, setCurrentPage, openConfirm]);

  // Widget operations
  const addWidget = useCallback((widgetData, pageId = currentPage) => {
    const libraryWidget = {
      id: generateId(),
      ...widgetData
    };
    setWidgetLibrary(prev => [...prev, libraryWidget]);
    
    const widget = {
      id: generateId(),
      libraryId: libraryWidget.id,
      ...widgetData
    };
    
    setWidgets(prev => [...prev, widget]);
    
    // Add widget to page
    if (pageId !== 'settings') {
      setPages(prev => prev.map(page => 
        page.id === pageId
          ? {
              ...page,
              widgets: [
                ...page.widgets,
                { id: widget.id, libraryId: widget.libraryId, x: 50, y: 50, width: 300, height: 200 }
              ]
            }
          : page
      ));
    }
    
    return widget;
  }, [currentPage, setWidgets, setWidgetLibrary, setPages]);

  const updateWidget = useCallback((widgetId, updates) => {
   
    setWidgetLibrary(prev => prev.map(widget => 
      widget.id === widgetId ? { ...widget, ...updates } : widget
    ));
    
    setWidgets(prev => prev.map(widget => 
      widget.id === widgetId ? { ...widget, ...updates } : widget
    ));

    setWidgets(prev => prev.map(widget => 
      widget.libraryId === widgetId ? { ...widget, ...updates, id: widget.id, libraryId: widget.libraryId } : widget
    ));

  }, [ setWidgetLibrary, setWidgets]);

  const deleteWidget = useCallback((widgetId) => {

    openConfirm('Are you sure you want to delete this widget?', () => {

      // Remove from all pages
      setPages(prev => prev.map(page => ({
        ...page,
        widgets: page.widgets.filter(w => w.libraryId !== widgetId)
      })));
      
      // Remove from widgets list
      setWidgets(prev => prev.filter(w => w.libraryId !== widgetId));

      // Remove from library
      setWidgetLibrary(prev => prev.filter(w => w.id !== widgetId));
    });

  }, [setPages, setWidgets, setWidgetLibrary, openConfirm]);

  const removeWidgetFromPage = useCallback((widgetId, pageId = currentPage) => {
    openConfirm('Are you sure you want to remove the widget from the page?', () => {
      setPages(prev => prev.map(page => 
        page.id === pageId
          ? {
              ...page,
              widgets: page.widgets.filter(w => w.id !== widgetId)
            }
          : page
      ));
    });

  }, [currentPage, setPages, openConfirm]); 

  const moveWidget = useCallback((widgetId, position, pageId = currentPage) => {
    setPages(prev => prev.map(page => 
      page.id === pageId
        ? {
            ...page,
            widgets: page.widgets.map(w => 
              w.id === widgetId ? { ...w, ...position } : w
            )
          }
        : page
    ));
  }, [currentPage, setPages]);
  
  const addFromLibrary = useCallback((libraryWidget, pageId = currentPage) => {
    const widget = {
      ...libraryWidget,
      id: generateId(),
      libraryId: libraryWidget.id
    };
    
    setWidgets(prev => [...prev, widget]);
    
    if (pageId !== 'settings') {
      setPages(prev => prev.map(page => 
        page.id === pageId
          ? {
              ...page,
              widgets: [
                ...page.widgets,
                { id: widget.id, libraryId: widget.libraryId, x: 50, y: 50, width: 300, height: 200 }
              ]
            }
          : page
      ));
    }
  }, [currentPage, setWidgets, setPages]);

  // Export/Import operations
  const exportConfig = useCallback(() => {
    const config = { pages, widgets, widgetLibrary };
    const blob = new Blob([JSON.stringify(config, null, 2)], { 
      type: 'application/json' 
    });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'dashboard-config.json';
    a.click();
    URL.revokeObjectURL(url);
  }, [pages, widgets, widgetLibrary]);

  const importConfig = useCallback(() => {
    const input = document.createElement('input');
    input.type = 'file';
    input.accept = 'application/json';
    input.onchange = (e) => {
      const file = e.target.files[0];
      if (!file) return;
      
      const reader = new FileReader();
      reader.onload = (event) => {
        try {
          const config = JSON.parse(event.target.result);
          
          if (config.pages) setPages(config.pages);
          if (config.widgets) setWidgets(config.widgets);
          if (config.widgetLibrary) setWidgetLibrary(config.widgetLibrary);
          
        } catch (error) {
          console.error('Import error:', error);
          alert('Invalid configuration file');
        }
      };
      reader.readAsText(file);
    };
    input.click();
  }, [setPages, setWidgets, setWidgetLibrary]);

  const value = {
    // State
    currentPage,
    isEditMode,
    pages,
    widgets,
    widgetLibrary,
    
    // State setters
    setCurrentPage,
    setIsEditMode,
    
    // Page operations
    addPage,
    updatePage,
    reorderPages,
    deletePage,
    
    // Widget operations
    addWidget,
    updateWidget,
    deleteWidget,
    removeWidgetFromPage,
    moveWidget,
    
    // Library operations
    addFromLibrary,
    
    // Config operations
    exportConfig,
    importConfig
  };

  return (
    <DashboardContext.Provider value={value}>
      
      <ConfirmModal
        isOpen={confirmModal.isOpen}
        message={confirmModal.message}
        title="Please Confirm"
        confirmLabel="Delete"
        cancelLabel="Cancel"
        onConfirm={() => {
          confirmModal.onConfirm?.();
          closeConfirm();
        }}
        onCancel={closeConfirm}
      />
      
      {children}

     

    </DashboardContext.Provider>
  
  );
};