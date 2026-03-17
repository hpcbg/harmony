import { useSelector } from "react-redux";
import DashboardPage from "./pages/DashboardPage.jsx";
import SettingsPage from "./pages/SettingsPage.jsx";
import DashboardLayout from "./components/layout/DashboardLayout.jsx";
import "./App.css";
import { selectPages } from "./store/pagesSlice.js";

function App() {
  const { currentPage } = useSelector(selectPages);
  const isSettingsPage = currentPage === "settings";

  return (
    <DashboardLayout>
      {isSettingsPage ? <SettingsPage /> : <DashboardPage />}
    </DashboardLayout>
  );
}

export default App;
