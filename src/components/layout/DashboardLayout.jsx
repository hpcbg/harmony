import { useSelector } from "react-redux";
import Sidebar from "../Sidebar";
import Header from "../Header";
import WidgetFormModal from "../modals/WidgetFormModal";
import PageFormModal from "../modals/PageFormModal";
import { selectPages } from "../../store/pagesSlice.js";

export default function DashboardLayout({ children }) {
  const { pages, currentPage } = useSelector(selectPages);

  const currentPageData = pages.find((p) => p.id === currentPage);
  const isSettingsPage = currentPage === "settings";
  const pageTitle = isSettingsPage ? "Settings" : currentPageData?.name;

  return (
    <div className="flex h-screen bg-gray-100">
      <Sidebar />

      <div className="flex-1 flex flex-col overflow-hidden">
        <Header title={pageTitle} />

        <div className="flex-1 overflow-hidden">{children}</div>
      </div>
      <WidgetFormModal />
      <PageFormModal />
    </div>
  );
}
