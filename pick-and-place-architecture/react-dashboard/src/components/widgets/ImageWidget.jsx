import { useState } from "react";
import { Image as ImageIcon, RefreshCw } from "lucide-react";

const ImageWidget = ({ widget, data }) => {
  const [imageError, setImageError] = useState(false);
  const [isLoading, setIsLoading] = useState(true);

  const imageUrl = data?.imageUrl || data?.url || data?.value;

  const handleImageLoad = () => {
    setIsLoading(false);
    setImageError(false);
  };

  const handleImageError = () => {
    setIsLoading(false);
    setImageError(true);
  };

  return (
    <div className="h-full flex flex-col bg-gradient-to-br from-pink-50 to-red-50 rounded p-4">
      <div className="text-large text-gray-600 mb-2 text-center flex justify-center">
        <span>{widget.name}</span>
        {isLoading && (
          <RefreshCw size={14} className="animate-spin text-gray-400" />
        )}
      </div>

      <div className="flex-1 flex items-center justify-center overflow-hidden rounded ">
        {!imageUrl ? (
          <div className="text-center text-gray-400">
            <ImageIcon size={48} className="mx-auto mb-2 opacity-50" />
            <p className="text-sm">No image available</p>
          </div>
        ) : imageError ? (
          <div className="text-center text-red-400">
            <ImageIcon size={48} className="mx-auto mb-2 opacity-50" />
            <p className="text-sm">Failed to load image</p>
            <p className="text-xs mt-1 text-gray-400">{imageUrl}</p>
          </div>
        ) : (
          <img
            src={imageUrl}
            alt={widget.name}
            className="max-w-full max-h-full object-contain"
            onLoad={handleImageLoad}
            onError={handleImageError}
            style={{ display: isLoading ? "none" : "block" }}
          />
        )}

        {isLoading && imageUrl && !imageError && (
          <div className="text-center text-gray-400">
            <RefreshCw
              size={48}
              className="mx-auto mb-2 opacity-50 animate-spin"
            />
            <p className="text-sm">Loading image...</p>
          </div>
        )}
      </div>
    </div>
  );
};

export default ImageWidget;
