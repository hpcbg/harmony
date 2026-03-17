import { createSlice } from "@reduxjs/toolkit";

const initialState = {
  editMode: false,
  pages: [
    { id: "home", name: "Home", widgets: {} },
    { id: "settings", name: "Settings", widgets: {} },
  ],
  currentPage: "home",
  editingPage: null,
  creatingNewPage: false,
};

const pagesSlice = createSlice({
  name: "pages",
  initialState,
  reducers: {
    toggleEditMode: (state) => {
      state.editMode = !state.editMode;
    },

    addPage: (state, action) => {
      state.pages.push(action.payload);
      state.creatingNewPage = false;
    },

    setPagesFromConfig: (state, action) => {
      state.pages = action.payload.map((page) => ({
        ...page,
        widgets: page.widgets || {},
      }));
      state.pages.push({ id: "settings", name: "Settings", widgets: {} });
      // Set current page to first page (or home)
      state.currentPage = state.pages[0]?.id || "home";
      state.editingPage = null;
      state.creatingNewPage = false;
    },

    startCreatePage(state) {
      state.creatingNewPage = true; // flag that we are creating
      state.editingPage = null;
    },

    editPageName: (state, action) => {
      const { id, name } = action.payload;
      const page = state.pages.find((p) => p.id === id);
      if (page) {
        page.name = name;
        state.editingPage = null;
      }
    },

    startEditPage(state, action) {
      state.editingPage = action.payload;
      state.creatingNewPage = false;
    },

    stopEditPage(state) {
      state.editingPage = null;
      state.creatingNewPage = false;
    },

    deletePage: (state, action) => {
      const pageId = action.payload;
      state.pages = state.pages.filter((p) => p.id !== pageId);
      if (state.currentPage === pageId) {
        state.currentPage = state.pages[0]?.id || "home";
      }
    },

    setCurrentPage: (state, action) => {
      state.currentPage = action.payload;
    },

    reorderPages: (state, action) => {
      state.pages = action.payload;
    },

    addWidgetToPage(state, action) {
      const { pageId, widgetId } = action.payload;
      const page = state.pages.find((p) => p.id === pageId);
      if (!page) return;

      page.widgets[widgetId] = {
        x: 50,
        y: 50,
        width: 300,
        height: 200,
      };
    },

    moveWidgetOnPage(state, action) {
      const { pageId, widgetId, position } = action.payload;
      const page = state.pages.find((p) => p.id === pageId);
      if (!page || !page.widgets[widgetId]) return;

      page.widgets[widgetId] = position;
    },

    removeWidgetFromPage(state, action) {
      const { pageId, widgetId } = action.payload;
      const page = state.pages.find((p) => p.id === pageId);
      if (!page) return;
      delete page.widgets[widgetId];
    },

    resizeWidget(state, action) {
      const { pageId, widgetId, position } = action.payload;
      const page = state.pages.find((p) => p.id === pageId);
      if (!page || !page.widgets[widgetId]) return;

      page.widgets[widgetId] = position;
    },
  },
});

export const {
  toggleEditMode,
  addPage,
  setPagesFromConfig,
  startCreatePage,
  editPageName,
  startEditPage,
  stopEditPage,
  deletePage,
  setCurrentPage,
  reorderPages,
  addWidgetToPage,
  moveWidgetOnPage,
  removeWidgetFromPage,
  resizeWidget,
} = pagesSlice.actions;

export default pagesSlice.reducer;

export const selectPages = (state) => state.pages;
