import { useState } from "react";
import { Power } from "lucide-react";

const ButtonWidget = ({ widget, onAction }) => {
  const [loading, setLoading] = useState(false);

  const handleClick = async () => {
    setLoading(true);
    await onAction(widget);
    setLoading(false);
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
};

export default ButtonWidget;
