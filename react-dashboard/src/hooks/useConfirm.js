import { useState, useCallback } from "react";

const useConfirm = () => {
  const [confirm, setConfirm] = useState(null);

  const openConfirm = useCallback((payload) => {
    setConfirm(payload);
  }, []);

  const closeConfirm = useCallback(() => {
    setConfirm(null);
  }, []);

  return {
    confirm,
    openConfirm,
    closeConfirm,
    isOpen: !!confirm,
  };
};

export default useConfirm;
