import { useState } from "react";
import { useDispatch } from "react-redux";
import { Power } from "lucide-react";
import { runButtonWidget } from "../../../store/widgetsThunks";

export default function ButtonWidget({ widget }) {
  const dispatch = useDispatch();
  const [loading, setLoading] = useState(false);

  const handleClick = async () => {
    setLoading(true);
    try {
      await dispatch(runButtonWidget(widget)).unwrap();
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="h-full flex flex-col items-center justify-center bg-orange-50 rounded p-4">
      <button
        onClick={handleClick}
        disabled={loading}
        className="px-6 py-3 bg-orange-500 text-white rounded-lg hover:bg-orange-600 disabled:opacity-50 flex items-center gap-2"
      >
        <Power size={20} />
        {loading ? "Processing..." : widget.name}
      </button>
    </div>
  );
}
