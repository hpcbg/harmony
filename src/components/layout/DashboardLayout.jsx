import React from 'react';
import Sidebar from './Sidebar';
import Header from './Header';

const DashboardLayout = ({ 
  pages,
  currentPage,
  isEditMode,
  onPageChange,
  onAddPage,
  onReorderPages,
  onAddWidget,
  onShowLibrary,
  children 
}) => {
  const currentPageData = pages.find(p => p.id === currentPage);
  const isSettingsPage = currentPage === 'settings';
  const pageTitle = isSettingsPage ? 'Settings' : currentPageData?.name || 'Dashboard';
  
  return (
    <div className="flex h-screen bg-gray-100">
      <Sidebar
        pages={pages}
        currentPage={currentPage}
        onPageChange={onPageChange}
        onAddPage={onAddPage}
        onReorderPages={onReorderPages}
        isEditMode={isEditMode}
      />
      
      <div className="flex-1 flex flex-col overflow-hidden">
        <Header
          title={pageTitle}
          isSettingsPage={isSettingsPage}
          isEditMode={isEditMode}
          onAddWidget={onAddWidget}
          onShowLibrary={onShowLibrary}
        />
        
        <div className="flex-1 overflow-hidden">
          {children}
        </div>
      </div>
    </div>
  );
};

export default DashboardLayout;