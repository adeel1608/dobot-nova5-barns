import React, { useState, useRef, useEffect } from 'react';
import NewPOSOrderForm from './NewPOSOrderForm';
import { DndContext, closestCenter } from '@dnd-kit/core';
import Swal from 'sweetalert2';
import {
  arrayMove,
  SortableContext,
  useSortable,
  verticalListSortingStrategy
} from '@dnd-kit/sortable';
import { CSS } from '@dnd-kit/utilities';
import useStore from '../../../store';
import backarrow from '../../../assets/backarrow.png';
import deleteIcon from '../../../assets/delete.png';

function SortableItem({ order, index, onStartOrder, onStopOrder, onResumeOrder, onDeleteOrder, onViewDetails, onReorderOrder, isStarting, isStopping, isResuming, isDeleting, isReordering, getStatusBadge }) {
  // Debug: Log that this component is rendering
  console.log(`📦 SortableItem rendering for order ${order.id} with status: ${order.status}`);
  
  const {
    attributes, listeners, setNodeRef,
    transform, transition, isDragging
  } = useSortable({ id: order.id });

  const style = {
    transform: CSS.Transform.toString(transform),
    transition,
    opacity: isDragging ? 0.5 : 1,
  };

  const disableDrag = index === 0 && (order.status === 'PROCESSING' || order.status === 'STOPPING');

  // Determine if buttons should be disabled (only COMPLETED orders are fully disabled)
  const isDisabled = order.status === 'COMPLETED';
  
  // Get status badge with colored dot
  const getStatusBadgeWithDot = (status) => {
    switch(status) {
      case 'PROCESSING':
        return (
          <span className="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-orange-100 text-orange-800">
            <div className="w-2 h-2 bg-orange-500 rounded-full mr-2"></div>
            Processing
          </span>
        );
      case 'STOPPING':
        return (
          <span className="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-amber-100 text-amber-800">
            <svg className="animate-spin w-3 h-3 mr-2" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24">
              <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
              <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
            </svg>
            Stopping...
          </span>
        );
      case 'COMPLETED':
        return (
          <span className="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-green-100 text-green-800">
            <div className="w-2 h-2 bg-green-600 rounded-full mr-2"></div>
            Completed
          </span>
        );
      case 'QUEUED':
        return (
          <span className="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-blue-100 text-blue-800">
            <div className="w-2 h-2 bg-blue-600 rounded-full mr-2"></div>
            Queued
          </span>
        );
      case 'CANCELLED':
      case 'ERROR':
      case 'STOPPED':
        return (
          <span className="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-red-100 text-red-800">
            <div className="w-2 h-2 bg-red-600 rounded-full mr-2"></div>
            {status === 'CANCELLED' ? 'Canceled' : status === 'ERROR' ? 'Error' : 'Stopped'}
          </span>
        );
      case 'HALTED':
        return (
          <span className="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-yellow-100 text-yellow-800">
            <div className="w-2 h-2 bg-yellow-500 rounded-full mr-2"></div>
            Halted
          </span>
        );
      default:
        return (
          <span className="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-gray-100 text-gray-800">
            <div className="w-2 h-2 bg-gray-500 rounded-full mr-2"></div>
            {status}
          </span>
        );
    }
  };

  // Visual highlight for processing/stopping orders
  const isProcessing = order.status === 'PROCESSING';
  const isInStoppingState = order.status === 'STOPPING';
  const containerClasses = isProcessing
    ? 'mb-2 p-4 rounded-lg border-2 border-orange-400 bg-orange-50 shadow-lg transition-all duration-200 ring-2 ring-orange-200'
    : isInStoppingState
    ? 'mb-2 p-4 rounded-lg border-2 border-amber-400 bg-amber-50 shadow-lg transition-all duration-200 ring-2 ring-amber-200'
    : 'mb-2 p-4 rounded-lg border border-gray-200 bg-white hover:shadow-sm transition-shadow duration-200';

  return (
    <div
      ref={setNodeRef}
      style={style}
      className={containerClasses}
    >
      <div className="flex items-center justify-between">
        {/* Left side - ID and Status */}
        <div className="flex flex-col space-y-2">
          <span className="text-md text-bold text-gray-900">Order ID: {order.id}</span>
          {getStatusBadgeWithDot(order.status)}
        </div>
        
        {/* Right side - Action Buttons */}
        <div className="flex items-center space-x-2">
          {/* Dynamic Action Button (Start/Stop/Resume) */}
          {order.status === 'PROCESSING' ? (
            // Stop Button for Processing Orders
            <button 
              type="button"
              onClick={() => onStopOrder(order.id)}
              disabled={isStopping === order.id}
              className={`px-3 py-1.5 rounded text-xs font-medium transition-colors ${
                isStopping === order.id
                  ? 'bg-red-300 text-white cursor-not-allowed'
                  : 'bg-red-600 text-white hover:bg-red-700'
              }`}
              title={isStopping === order.id ? "Stopping..." : "Stop processing"}
            >
              {isStopping === order.id ? "Stopping..." : "Stop"}
            </button>
          ) : order.status === 'STOPPING' ? (
            // Stopping - button disabled while waiting
            <button 
              type="button"
              disabled={true}
              className="px-3 py-1.5 rounded text-xs font-medium bg-amber-300 text-white cursor-not-allowed"
              title="Stopping - waiting for current task to complete"
            >
              <span className="inline-flex items-center">
                <svg className="animate-spin h-3 w-3 mr-1" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24">
                  <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
                  <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
                </svg>
                Stopping...
              </span>
            </button>
          ) : order.status === 'STOPPED' ? (
            // Resume Button for Stopped Orders
            <button 
              type="button"
              onClick={() => onResumeOrder(order.id)}
              disabled={isResuming === order.id}
              className={`px-3 py-1.5 rounded text-xs font-medium transition-colors ${
                isResuming === order.id
                  ? 'bg-blue-300 text-white cursor-not-allowed'
                  : 'text-white'
              }`}
              style={{
                backgroundColor: isResuming === order.id ? undefined : '#00754A'
              }}
              title={isResuming === order.id ? "Resuming..." : "Resume processing"}
            >
              {isResuming === order.id ? "Resuming..." : "Resume"}
            </button>
          ) : (
            // Start Button for Queued/Cancelled/Error Orders
            <button 
              type="button"
              onClick={() => onStartOrder(order.id)}
              disabled={isDisabled || isStarting === order.id}
              className={`px-3 py-1.5 rounded text-xs font-medium transition-colors ${
                isDisabled 
                  ? 'bg-gray-300 text-gray-500 cursor-not-allowed'
                  : isStarting === order.id
                    ? 'bg-blue-300 text-white cursor-not-allowed'
                    : 'text-white'
              }`}
              style={{
                backgroundColor: isDisabled 
                  ? undefined 
                  : isStarting === order.id 
                    ? undefined 
                    : '#00754A'
              }}
              title={isDisabled ? "Action not available" : "Start processing"}
            >
              {isStarting === order.id ? "Starting..." : "Start"}
            </button>
          )}

          {/* Details Button - Always enabled so users can view completed orders */}
          <button 
            type="button"
            onClick={() => onViewDetails(order)}
            className="px-3 py-1.5 rounded text-xs font-medium border-2 transition-colors border-green-600 text-green-600 hover:bg-green-50"
            style={{
              borderWidth: '2px',
              borderStyle: 'solid',
              borderColor: '#059669'
            }}
            title="View order details"
          >
            Details
          </button>

            {/* Reorder Button - outlined like Details */}
            <button 
              type="button"
              onClick={() => onReorderOrder(order)}
              disabled={isReordering === order.id}
              className={`px-3 py-1.5 rounded text-xs font-medium border-2 transition-colors ${
                isReordering === order.id 
                  ? 'border-green-300 text-green-300 cursor-not-allowed'
                  : 'border-green-600 text-green-600 hover:bg-green-50'
              }`}
              style={{
                borderWidth: '2px',
                borderStyle: 'solid',
                borderColor: isReordering === order.id ? '#86efac' : '#059669'
              }}
              title={isReordering === order.id ? 'Reordering...' : 'Reorder this order'}
            >
              {isReordering === order.id ? (
                <span className="inline-flex items-center">
                  <svg className="animate-spin h-3 w-3 mr-1" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24">
                    <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
                    <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
                  </svg>
                  Reordering...
                </span>
              ) : (
                <span className="inline-flex items-center">
                 
                  Reorder
                </span>
              )}
            </button>

            {/* Delete Button - Icon only */}
            <button 
              type="button"
              onClick={() => {
                console.log('🗑️ Delete button clicked for order:', order.id);
                onDeleteOrder && onDeleteOrder(order.id);
              }}
              disabled={isDeleting}
              className={`text-xs px-2 py-1 rounded flex items-center justify-center  text-white hover:opacity-90 disabled:opacity-60 disabled:cursor-not-allowed shadow-none`}
              title={isDeleting ? 'Deleting...' : 'Delete order'}
              aria-label={isDeleting ? 'Deleting...' : 'Delete order'}
              style={{height:'2rem',boxShadow:'none'}}
            >
              {isDeleting ? (
                <svg className="animate-spin h-3 w-3" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24">
                  <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
                  <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
                </svg>
              ) : (
                <img src={deleteIcon} alt="" aria-hidden="true" className="w-10 h-10" />
              )}
            </button>
          </div>
        </div>
      </div>
  );
}

function OrderQueue({ connectionStatus }) {

  const { 
    orders,
    ordersTotal,
    ordersHasMore,
    menuItems,
    ingredientsByCategory,
    sendReorder,
    createOrder,
    startOrder,
    stopOrder,
    resumeOrder, 
    deleteOrder, 
    processPOSOrder,
    loadMoreOrders,
    fetchMenuItems,
    fetchIngredientsByCategory,
    isLoading, 
    errors, 
    clearError 
  } = useStore();
  
  const [selectedOrder, setSelectedOrder] = useState(null);
  const [filterStatus, setFilterStatus] = useState('ALL');
  const [searchTerm, setSearchTerm] = useState('');
  const [startingOrderId, setStartingOrderId] = useState(null);
  const [stoppingOrderId, setStoppingOrderId] = useState(null);
  const [resumingOrderId, setResumingOrderId] = useState(null);
  const [deletingOrderId, setDeletingOrderId] = useState(null);
  const [reorderingOrderId, setReorderingOrderId] = useState(null);
  const [showNewOrder, setShowNewOrder] = useState(false);
  const [showOrderDetails, setShowOrderDetails] = useState(false);
  const [posOrderData, setPosOrderData] = useState({
    items: [{ 
      item_id: '', 
      quantity: 1,
      kitchen_notes: [],
      item_ingredients: [],
      selectedDrinkName: '',
      selectedSize: '',
      isExpanded: false,
      isCustomizeOpen: false
    }]
  });
  const logsEndRef = useRef(null);
  const scrollContainerRef = useRef(null);

  // Fetch menu items and ingredients for POS mode
  useEffect(() => {
    if (showNewOrder) {
      if (menuItems.length === 0) {
        fetchMenuItems();
      }
      if (Object.keys(ingredientsByCategory).length === 0) {
        fetchIngredientsByCategory();
      }
    }
  }, [showNewOrder, menuItems.length, ingredientsByCategory, fetchMenuItems, fetchIngredientsByCategory]);

  const displayOrders = orders;

  const getStatusBadge = (status) => {
    switch(status) {
      case 'PROCESSING':
        return <span className="bg-yellow-100 text-yellow-800 text-xs font-medium px-2.5 py-0.5 rounded">🔄 Processing</span>;
      case 'STOPPING':
        return <span className="bg-amber-100 text-amber-800 text-xs font-medium px-2.5 py-0.5 rounded">⏳ Stopping...</span>;
      case 'COMPLETED':
        return <span className="bg-green-100 text-green-800 text-xs font-medium px-2.5 py-0.5 rounded">✅ Completed</span>;
      case 'HALTED':
        return <span className="bg-orange-100 text-orange-800 text-xs font-medium px-2.5 py-0.5 rounded">⏸️ Halted</span>;
      case 'STOPPED':
        return <span className="bg-red-100 text-red-800 text-xs font-medium px-2.5 py-0.5 rounded">🛑 Stopped</span>;
      case 'ERROR':
        return <span className="bg-red-100 text-red-800 text-xs font-medium px-2.5 py-0.5 rounded">❌ Error</span>;
      case 'CANCELLED':
        return <span className="bg-gray-100 text-gray-800 text-xs font-medium px-2.5 py-0.5 rounded">🚫 Cancelled</span>;
      default:
        return <span className="bg-blue-100 text-blue-800 text-xs font-medium px-2.5 py-0.5 rounded">📋 Queued</span>;
    }
  };

  const handleDragEnd = ({ active, over }) => {
    if (!over || errors.orders) return; // Prevent reordering if there's an API error
    
    const oldIndex = displayOrders.findIndex(o => o.id === active.id);
    const newIndex = displayOrders.findIndex(o => o.id === over.id);
    if (oldIndex === 0 && (displayOrders[0].status === 'PROCESSING' || displayOrders[0].status === 'STOPPING')) return; // block moving processing/stopping
    
    const newOrders = arrayMove(displayOrders, oldIndex, newIndex);
    sendReorder(newOrders);
  };

  const handleStartOrder = async (orderId) => {
    console.log('🚀 handleStartOrder called with orderId:', orderId);
    console.log('🚀 Current startingOrderId:', startingOrderId);
    
    setStartingOrderId(orderId);
    try {
      console.log('🚀 About to call startOrder from store...');
      const success = await startOrder(orderId);
      console.log('🚀 startOrder returned:', success);
      
      if (success) {
        console.log(`✅ Order ${orderId} started successfully`);
        // Show success feedback
        setTimeout(() => {
          setStartingOrderId(null);
        }, 1000); // Keep loading state for 1 second to show feedback
      } else {
        console.error(`❌ Failed to start order ${orderId}`);
        setStartingOrderId(null);
      }
    } catch (error) {
      console.error('🚀 Error in handleStartOrder:', error);
      setStartingOrderId(null);
    }
  };

  const handleStopOrder = async (orderId) => {
    console.log('🛑 handleStopOrder called with orderId:', orderId);
    
    setStoppingOrderId(orderId);
    try {
      console.log('🛑 About to call stopOrder from store...');
      const success = await stopOrder(orderId);
      console.log('🛑 stopOrder returned:', success);
      
      if (success) {
        console.log(`✅ Order ${orderId} stopped successfully`);
        setTimeout(() => {
          setStoppingOrderId(null);
        }, 1000);
      } else {
        console.error(`❌ Failed to stop order ${orderId}`);
        setStoppingOrderId(null);
      }
    } catch (error) {
      console.error('🛑 Error in handleStopOrder:', error);
      setStoppingOrderId(null);
    }
  };

  const handleResumeOrder = async (orderId) => {
    console.log('🔄 handleResumeOrder called with orderId:', orderId);
    
    setResumingOrderId(orderId);
    try {
      console.log('🔄 About to call resumeOrder from store...');
      const success = await resumeOrder(orderId);
      console.log('🔄 resumeOrder returned:', success);
      
      if (success) {
        console.log(`✅ Order ${orderId} resumed successfully`);
        setTimeout(() => {
          setResumingOrderId(null);
        }, 1000);
      } else {
        console.error(`❌ Failed to resume order ${orderId}`);
        setResumingOrderId(null);
      }
    } catch (error) {
      console.error('🔄 Error in handleResumeOrder:', error);
      setResumingOrderId(null);
    }
  };

  const handleCalibrateOrder = async () => {
    try {
      // Create order using direct cups format since Calibrate is a manual item
      const calibrateOrder = {
        cups: [{
          type: 'Calibrate',
          size: '9oz',
          addons: [],
          ingredients: []
        }]
      };
      
      const success = await createOrder(calibrateOrder);
      
      if (success) {
        Swal.fire({
          icon: 'success',
          title: 'Calibrate Order Created!',
          text: 'Calibration order has been added to the queue.',
          timer: 2500,
          timerProgressBar: true,
          showConfirmButton: false,
        });
      } else {
        Swal.fire({
          icon: 'error',
          title: 'Failed!',
          text: 'Could not create calibration order.',
          timer: 2500,
          timerProgressBar: true,
          showConfirmButton: false,
        });
      }
    } catch (error) {
      console.error('Error creating calibrate order:', error);
      Swal.fire({
        icon: 'error',
        title: 'Error',
        text: 'An unexpected error occurred while creating calibration order.',
        timer: 2500,
        timerProgressBar: true,
        showConfirmButton: false,
      });
    }
  };

  const handleReorderOrder = async (order) => {
    try {
      const cups = (order.cups || []).map(cup => ({
        type: cup.type || cup.drink_type || '',
        size: cup.size || cup.cup_size || 'regular',
        addons: Array.isArray(cup.addons) ? cup.addons : [],
        ingredients: cup.ingredients || []
      })).filter(c => c.type && c.type.trim() !== '');

      if (cups.length === 0) {
        Swal.fire({
          icon: 'warning',
          title: 'Cannot Reorder',
          text: 'Original order has no valid cups to reorder.',
          timer: 2500,
          timerProgressBar: true,
          showConfirmButton: false,
        });
        return;
      }

      setReorderingOrderId(order.id);
      const success = await createOrder({ cups });

      if (success) {
        Swal.fire({
          icon: 'success',
          title: 'Reordered!',
          text: `A new order has been created from #${order.id}.`,
          timer: 2500,
          timerProgressBar: true,
          showConfirmButton: false,
        });
      } else {
        Swal.fire({
          icon: 'error',
          title: 'Failed to Reorder',
          text: 'Could not create a new order from this one.',
          timer: 2500,
          timerProgressBar: true,
          showConfirmButton: false,
        });
      }
    } catch (e) {
      console.error('Error in handleReorderOrder:', e);
      Swal.fire({
        icon: 'error',
        title: 'Error',
        text: 'An unexpected error occurred while reordering.',
        timer: 2500,
        timerProgressBar: true,
        showConfirmButton: false,
      });
    } finally {
      setReorderingOrderId(null);
    }
  };

const handleDeleteOrder = async (orderId) => {
  console.log('🗑️ handleDeleteOrder called with orderId:', orderId);
  
  // Find the order to get its status
  const order = displayOrders.find(o => o.id === orderId);
  const orderStatus = order?.status?.toUpperCase() || 'UNKNOWN';

  // Determine SweetAlert config
  let swalConfig = {
    title: `Delete Order #${orderId}?`,
    icon: 'warning',
    showCancelButton: true,
    confirmButtonText: 'Yes, delete it!',
    cancelButtonText: 'Cancel',
    reverseButtons: true,
  };

  if (orderStatus === 'PROCESSING') {
    swalConfig = {
      ...swalConfig,
      title: `⚠️ Order #${orderId} is Processing!`,
      text: `Deleting this order will immediately stop all ongoing operations and may cause system issues. Are you absolutely sure you want to force delete it?`,
    };
  } else {
    swalConfig = {
      ...swalConfig,
      text: `This action cannot be undone.`,
    };
  }

  const result = await Swal.fire(swalConfig);

  if (!result.isConfirmed) return;

  setDeletingOrderId(orderId);

  try {
    console.log('🗑️ About to call deleteOrder from store...');
    const success = await deleteOrder(orderId);
    console.log('🗑️ deleteOrder returned:', success);

    if (success) {
      Swal.fire({
        icon: 'success',
        title: 'Deleted!',
        text: `Order #${orderId} has been deleted.`,
        timer: 3000,
        timerProgressBar: true,
        showConfirmButton: false,
      });
    } else {
      Swal.fire({
        icon: 'error',
        title: 'Failed!',
        text: `Could not delete order #${orderId}.`,
        timer: 3000,
        timerProgressBar: true,
        showConfirmButton: false,
      });
    }
  } catch (error) {
    console.error('🗑️ Error in handleDeleteOrder:', error);
    Swal.fire({
      icon: 'error',
      title: 'Error!',
      text: 'An unexpected error occurred while deleting the order.',
      timer: 3000,
      timerProgressBar: true,
      showConfirmButton: false,
    });
  } finally {
    setDeletingOrderId(null);
  }
};

  const viewOrderDetails = (order) => {
    setSelectedOrder(order);
    setShowOrderDetails(true);
  };

  const closeOrderDetails = () => {
    setSelectedOrder(null);
    setShowOrderDetails(false);
  };

  const retryFetchOrders = () => {
    clearError('orders');
    useStore.getState().fetchOrders();
  };

  const handleLoadMore = async () => {
    // Save current scroll position before loading
    const scrollContainer = scrollContainerRef.current;
    if (!scrollContainer) {
      await loadMoreOrders();
      return;
    }

    const scrollTop = scrollContainer.scrollTop;
    
    // Load more orders
    await loadMoreOrders();
    
    // Restore scroll position after React re-renders
    // Use setTimeout to ensure DOM has fully updated
    setTimeout(() => {
      if (scrollContainer) {
        // Maintain the same scroll position so user stays looking at the same content
        scrollContainer.scrollTop = scrollTop;
      }
    }, 0);
  };

  // POS Order Functions
  const handleSubmitPOSOrder = async (e) => {
    e.preventDefault();
    
    // Validate that all items have item_id
    const invalidItems = posOrderData.items.filter(item => !item.item_id || item.item_id.trim() === '');
    if (invalidItems.length > 0) {
      Swal.fire({
        title: 'Missing Item ID',
        text: 'Please enter an item ID for all items before creating the order.',
        icon: 'warning',
        timer: 3000,
        timerProgressBar: true,
        showConfirmButton: false
      });
      return;
    }
    
    // Generate transaction_id
    const transactionId = `TXN${Date.now()}`;
    const now = new Date();
    const date = now.toISOString().split('T')[0];
    const time = now.toTimeString().split(' ')[0];
    
    // Build POS order format matching desired_output.json
    const posOrder = {
      transaction_id: transactionId,
      date: date,
      time: time,
      store_number: 1,
      pos_reg_id: 1,
      customer_id: null,
      items: posOrderData.items
    };
    
    const success = await processPOSOrder(posOrder);
    if (success) {
      setPosOrderData({ 
        items: [{ 
          item_id: '', 
          quantity: 1,
          kitchen_notes: [],
          item_ingredients: [],
          selectedDrinkName: '',
          selectedSize: ''
        }]
      });
      setShowNewOrder(false);

      Swal.fire({
        title: 'Success!',
        text: 'POS Order processed successfully!',
        icon: 'success',
        timer: 3000,
        timerProgressBar: true,
        showConfirmButton: false
      });
    } else {
      Swal.fire({
        title: 'Failed!',
        text: 'Failed to process POS order!',
        icon: 'error',
        timer: 3000,
        timerProgressBar: true,
        showConfirmButton: false
      });
    }
  };

  const addPOSItem = () => {
    setPosOrderData(prev => ({
      items: [...prev.items, { 
        item_id: '', 
        quantity: 1,
        kitchen_notes: [],
        item_ingredients: [],
        selectedDrinkName: '',
        selectedSize: '',
        isExpanded: true,
        isCustomizeOpen: false
      }]
    }));
  };

  const removePOSItem = (index) => {
    if (posOrderData.items.length > 1) {
      setPosOrderData(prev => ({
        items: prev.items.filter((_, i) => i !== index)
      }));
    }
  };

  const updatePOSItem = (index, field, value) => {
    setPosOrderData(prev => ({
      items: prev.items.map((item, i) => {
        if (i !== index) return item;

        // Two-step selection logic
        if (field === 'selectedDrinkName') {
          return {
            ...item,
            selectedDrinkName: value,
            selectedSize: '',
            item_id: '',
            selectedMenuItem: undefined,
            kitchen_notes: [],
            item_ingredients: [],
            isExpanded: true,
            isCustomizeOpen: false
          };
        }

        if (field === 'selectedSize') {
          const drinkName = item.selectedDrinkName;
          const selectedMenuItem = menuItems.find(m => m.name === drinkName && m.size === value);
          if (selectedMenuItem) {
            return {
              ...item,
              selectedSize: value,
              item_id: selectedMenuItem.item_id,
              selectedMenuItem: selectedMenuItem,
              kitchen_notes: [],
              item_ingredients: [],
              isExpanded: true,
              isCustomizeOpen: false
            };
          }
          return { ...item, selectedSize: value, item_id: '', selectedMenuItem: undefined };
        }

        // Fallback: item_id direct change
        if (field === 'item_id') {
          const selectedMenuItem = menuItems.find(m => m.item_id === value);
          if (selectedMenuItem) {
            return {
              ...item,
              item_id: value,
              selectedDrinkName: selectedMenuItem.name || item.selectedDrinkName || '',
              selectedSize: selectedMenuItem.size || item.selectedSize || '',
              selectedMenuItem: selectedMenuItem,
              kitchen_notes: [],
              item_ingredients: []
            };
          }
        }

        return { ...item, [field]: value };
      })
    }));
  };

  const addKitchenNote = (itemIndex) => {
    setPosOrderData(prev => ({
      items: prev.items.map((item, i) => 
        i === itemIndex ? { 
          ...item, 
          kitchen_notes: [...item.kitchen_notes, { type: '', qty: 1, detail: '' }]
        } : item
      )
    }));
  };

  const removeKitchenNote = (itemIndex, noteIndex) => {
    setPosOrderData(prev => ({
      items: prev.items.map((item, i) => 
        i === itemIndex ? {
          ...item,
          kitchen_notes: item.kitchen_notes.filter((_, ni) => ni !== noteIndex)
        } : item
      )
    }));
  };

  const updateKitchenNote = (itemIndex, noteIndex, field, value) => {
    setPosOrderData(prev => ({
      items: prev.items.map((item, i) => 
        i === itemIndex ? {
          ...item,
          kitchen_notes: item.kitchen_notes.map((note, ni) => 
            ni === noteIndex ? { ...note, [field]: value } : note
          )
        } : item
      )
    }));
  };

  const addIngredientModification = (itemIndex) => {
    setPosOrderData(prev => ({
      items: prev.items.map((item, i) => 
        i === itemIndex ? {
          ...item,
          item_ingredients: [...item.item_ingredients, { 
            itemId: '', 
            qty: 1, 
            isAddon: false,
            modifierGroupId: '',
            isModified: false,
            initialItemId: ''
          }]
        } : item
      )
    }));
  };

  const removeIngredientModification = (itemIndex, ingIndex) => {
    setPosOrderData(prev => ({
      items: prev.items.map((item, i) => 
        i === itemIndex ? {
          ...item,
          item_ingredients: item.item_ingredients.filter((_, ii) => ii !== ingIndex)
        } : item
      )
    }));
  };

  const updateIngredientModification = (itemIndex, ingIndex, field, value) => {
    setPosOrderData(prev => ({
      items: prev.items.map((item, i) => 
        i === itemIndex ? {
          ...item,
          item_ingredients: item.item_ingredients.map((ing, ii) => 
            ii === ingIndex ? { ...ing, [field]: value } : ing
          )
        } : item
      )
    }));
  };

  // Helper function to add an ingredient replacement
  const addIngredientReplacement = (itemIndex, originalIngredientId, newIngredientId, quantity, modifierGroupId) => {
    setPosOrderData(prev => ({
      items: prev.items.map((item, i) => 
        i === itemIndex ? {
          ...item,
          item_ingredients: [...item.item_ingredients, {
            itemId: newIngredientId,
            qty: quantity || 1,
            isAddon: false,
            modifierGroupId: modifierGroupId || 'replacement',
            isModified: true,
            initialItemId: originalIngredientId
          }]
        } : item
      )
    }));
  };

  // Helper function to add an ingredient addon
  const addIngredientAddon = (itemIndex, ingredientId, quantity) => {
    setPosOrderData(prev => ({
      items: prev.items.map((item, i) => 
        i === itemIndex ? {
          ...item,
          item_ingredients: [...item.item_ingredients, {
            itemId: ingredientId,
            qty: quantity || 1,
            isAddon: true,
            modifierGroupId: 'AddOns'
          }]
        } : item
      )
    }));
  };

  // Lookup ingredient name by id from ingredientsByCategory
  const getIngredientNameById = (id) => {
    for (const arr of Object.values(ingredientsByCategory)) {
      const found = arr.find(x => String(x.ingredient_id) === String(id));
      if (found) return found.name || found.type || String(id);
    }
    return String(id);
  };

  // Lightweight details getter used to check category when listing current add-ons
  const getIngredientDetailsById = (id) => {
    for (const [category, arr] of Object.entries(ingredientsByCategory)) {
      const found = arr.find(x => String(x.ingredient_id) === String(id));
      if (found) return { ...found, category };
    }
    return null;
  };

  const formatOrderForDisplay = (order) => {
    return {
      ...order,
      // Map cup data to display format
      itemName: order.cups && order.cups.length > 0 
        ? order.cups.map(cup => `${cup.drink_type || cup.type} (${cup.cup_size || cup.size})`).join(', ')
        : 'Unknown Item',
      // Map status
      status: order.status?.toUpperCase() || 'QUEUED',
      // Check if manual intervention is required
      manualRequired: order.cups && order.cups.some(cup => 
        cup.addons && cup.addons.includes('manual_required')
      ),
      // Format timestamps
      createdAt: order.created_at ? new Date(order.created_at).toLocaleString() : 'N/A',
      startedAt: order.started_at ? new Date(order.started_at).toLocaleString() : 'N/A',
      completedAt: order.completed_at ? new Date(order.completed_at).toLocaleString() : 'N/A',
      // For "Updated" field, show the most recent timestamp
      updatedAt: order.completed_at ? new Date(order.completed_at).toLocaleString() : 
                 order.started_at ? new Date(order.started_at).toLocaleString() : 'N/A'
    };
  };

  // Filter orders based on status and search term
  const filteredOrders = displayOrders.map(formatOrderForDisplay).filter(order => {
    // Filter by status
    if (filterStatus !== 'ALL' && order.status !== filterStatus) {
      return false;
    }
    
    // Filter by search term
    if (searchTerm && !order.itemName.toLowerCase().includes(searchTerm.toLowerCase())) {
      return false;
    }
    
    return true;
  });

  // Debug: Log filtered orders to see what we're working with
  console.log('🎯 Filtered orders:', filteredOrders);

  // Build unique drink list for two-step selection
  const uniqueDrinkNames = React.useMemo(() => {
    const names = menuItems.map(m => m.name).filter(Boolean);
    return Array.from(new Set(names));
  }, [menuItems]);

  return (
    <div className="bg-white rounded-lg shadow-xl  flex flex-col h-full">
      {/* Header */}
      <div className="flex flex-col space-y-3 p-3 md:p-4 border-b border-gray-200 flex-shrink-0">
        <div className="flex flex-col sm:flex-row sm:items-center justify-between space-y-2 sm:space-y-0">
          <div className="flex items-center">
            <h2 className="text-lg md:text-xl font-bold flex items-center ">
              {showNewOrder ? 'New Order' : 'Order Queue'}
              <div
                className={`w-2 h-2 rounded-full mx-3 ${
                  connectionStatus ? 'bg-green-500' : 'bg-red-500'
                }`}
              ></div>
            </h2>
            {errors.orders && (
              <span className="ml-2 inline-flex items-center px-2 py-1 rounded-full text-xs font-medium bg-red-100 text-red-800">
                API Error
              </span>
            )}
          </div>
          
          <div className="flex items-center gap-2">
            <h2
              onClick={handleCalibrateOrder}
              className="text-sm rounded font-small transition-colors duration-300 px-4 py-2 cursor-pointer barns-dark-bg text-white hover:barns-bg"
              style={{color:'white'}}
            >
              Calibrate
            </h2>
            
            <h2
              onClick={() => setShowNewOrder(!showNewOrder)}
              className={`text-sm rounded font-small transition-colors duration-300 px-4 py-2 cursor-pointer
                ${
                  showNewOrder
                    ? 'bg-gray-100 text-white hover:bg-gray-200 button-sm'
                    : 'barns-dark-bg text-white hover:barns-bg'
                }`}
              style={{color:'white'}}
            >
              {showNewOrder ? 'View Orders' : 'New Order'}
            </h2>
          </div>
        </div>
        
        {!showNewOrder && (
          <div className="flex flex-col sm:flex-row space-y-2 sm:space-y-0 sm:space-x-3">
            <input
              type="text"
              placeholder="Search orders..."
              value={searchTerm}
              onChange={(e) => setSearchTerm(e.target.value)}
              className="flex-1 px-3 py-2 barns-border-dark   focus:ring-0  text-sm"
            />
            <select
              value={filterStatus}
              onChange={(e) => setFilterStatus(e.target.value)}
              className="px-4 py-2 barns-border-dark focus:ring-2  text-sm sm:w-auto"
            >
              <option value="ALL">All Orders</option>
              <option value="QUEUED">Queued</option>
              <option value="PROCESSING">Processing</option>
              <option value="STOPPING">Stopping</option>
              <option value="HALTED">Halted</option>
              <option value="COMPLETED">Completed</option>
              <option value="STOPPED">Stopped</option>
              <option value="ERROR">Error</option>
              <option value="CANCELLED">Cancelled</option>
            </select>
            
          </div>
        )}
      </div>
      
      {/* API Error display */}
      {errors.orders && (
        <div className="border-b border-red-200 bg-red-50 px-4 py-2 text-sm text-red-700 flex justify-between items-center flex-shrink-0">
          <div>
            <span className="font-medium">API Error:</span> {errors.orders}
            <p className="text-xs mt-1">Please check the connection to the OMS service.</p>
          </div>
          <button 
            onClick={retryFetchOrders}
            className="px-2 py-1 bg-red-100 hover:bg-red-200 text-red-800 rounded text-xs font-medium"
          >
            Retry
          </button>
        </div>
      )}

      {/* Scrollable content area */}
      <div className="flex-1 overflow-hidden">
        <div ref={scrollContainerRef} className="h-full overflow-y-auto p-4">
          {showNewOrder ? (
            <NewPOSOrderForm
              posOrderData={posOrderData}
              setPosOrderData={setPosOrderData}
              menuItems={menuItems}
              ingredientsByCategory={ingredientsByCategory}
              isLoading={isLoading}
              uniqueDrinkNames={uniqueDrinkNames}
              handleSubmitPOSOrder={handleSubmitPOSOrder}
              addPOSItem={addPOSItem}
              removePOSItem={removePOSItem}
              updatePOSItem={updatePOSItem}
              addKitchenNote={addKitchenNote}
              updateKitchenNote={updateKitchenNote}
              removeKitchenNote={removeKitchenNote}
              addIngredientReplacement={addIngredientReplacement}
              addIngredientAddon={addIngredientAddon}
              removeIngredientModification={removeIngredientModification}
              getIngredientDetailsById={getIngredientDetailsById}
              getIngredientNameById={getIngredientNameById}
              onCancel={() => {
                setShowNewOrder(false);
                setPosOrderData({
                  items: [{
                    item_id: '',
                    quantity: 1,
                    kitchen_notes: [],
                    item_ingredients: [],
                    selectedDrinkName: '',
                    selectedSize: '',
                    isExpanded: false,
                    isCustomizeOpen: false
                  }]
                });
              }}
            />
          ) : (
            /* Orders List */
            <>
              {isLoading ? (
                <div className="flex justify-center items-center p-8">
                  <svg className="animate-spin h-8 w-8 text-blue-500" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24">
                    <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
                    <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
                  </svg>
                </div>
              ) : filteredOrders.length === 0 ? (
                <div className="text-center p-8 text-gray-500">
                  {searchTerm || filterStatus !== 'ALL' ? (
                    <p>No orders match your filters.</p>
                  ) : errors.orders ? (
                    <p>Unable to load orders. Please check the connection.</p>
                  ) : (
                    <p>No orders in the queue.</p>
                  )}
                </div>
              ) : (
                <DndContext collisionDetection={closestCenter} onDragEnd={handleDragEnd}>
                  <SortableContext items={filteredOrders.map(o => o.id)} strategy={verticalListSortingStrategy}>
                    <div className="space-y-2">
                      {filteredOrders.map((order, idx) => (
                        <SortableItem 
                          key={order.id} 
                          order={order} 
                          index={idx} 
                          onStartOrder={handleStartOrder}
                          onStopOrder={handleStopOrder}
                          onResumeOrder={handleResumeOrder}
                          onDeleteOrder={handleDeleteOrder}
                          onViewDetails={viewOrderDetails}
                          onReorderOrder={handleReorderOrder}
                          isStarting={startingOrderId === order.id}
                          isStopping={stoppingOrderId === order.id}
                          isResuming={resumingOrderId === order.id}
                          isDeleting={deletingOrderId === order.id}
                          isReordering={reorderingOrderId === order.id}
                          getStatusBadge={getStatusBadge}
                        />
                      ))}
                    </div>
                  </SortableContext>
                </DndContext>
              )}
              
              {/* Show More Button */}
              {!showNewOrder && filteredOrders.length > 0 && ordersHasMore && (
                <div className="mt-4 flex justify-center">
                  <button
                    onClick={handleLoadMore}
                    disabled={isLoading}
                    className="px-6 py-3 bg-gray-100 hover:bg-gray-200 text-gray-700 rounded-lg text-sm font-medium transition-all duration-200 disabled:opacity-50 disabled:cursor-not-allowed flex items-center space-x-2"
                  >
                    {isLoading ? (
                      <>
                        <svg className="animate-spin h-4 w-4" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24">
                          <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
                          <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
                        </svg>
                        <span>Loading...</span>
                      </>
                    ) : (
                      <>
                        <svg className="w-4 h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 9l-7 7-7-7" />
                        </svg>
                        <span>Show More ({ordersTotal - orders.length} remaining)</span>
                      </>
                    )}
                  </button>
                </div>
              )}
            </>
          )}
        </div>
      </div>

      {/* Order Details Modal */}
      {selectedOrder && (




        <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50 p-4">
          <div className="bg-white rounded-xl max-w-4xl w-full max-h-[90vh] overflow-y-auto shadow-2xl">
            {/* Header */}
            <div className="flex justify-between items-center p-6 border-b border-gray-200 bg-gradient-to-r from-blue-50 to-indigo-50">
              <div className="flex items-center space-x-4">
                <div className="p-2 bg-blue-500 rounded-lg">
                  <svg className="w-6 h-6 text-white" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 12h6m-6 4h6m2 5H7a2 2 0 01-2-2V5a2 2 0 012-2h5.586a1 1 0 01.707.293l5.414 5.414a1 1 0 01.293.707V19a2 2 0 01-2 2z" />
                  </svg>
                </div>
                <div>
                  <h2 className="text-2xl font-bold text-gray-900">Order #{selectedOrder.id}</h2>
                  <p className="text-gray-600">{selectedOrder.itemName}</p>
                </div>
              </div>
              
              <div className="flex items-center space-x-4">
                {getStatusBadge(selectedOrder.status)}
                <button
                  onClick={closeOrderDetails}
                  className="p-2 hover:bg-gray-100 rounded-lg transition-colors"
                >
                  <svg className="w-6 h-6 text-gray-400" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
                  </svg>
                </button>
              </div>
            </div>
            
            {/* Body */}
            <div className="p-6">
              <div className="grid grid-cols-1 lg:grid-cols-2 gap-8">
                {/* Basic Information */}
                <div className="space-y-6">
                  <div>
                    <h3 className="text-lg font-semibold text-gray-900 mb-4 flex items-center">
                      <svg className="w-5 h-5 mr-2 text-blue-500" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 16h-1v-4h-1m1-4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
                      </svg>
                      Order Information
                    </h3>
                    <div className="bg-gray-50 rounded-lg p-4 space-y-3">
                      <div className="flex justify-between">
                        <span className="text-gray-600 font-medium">Order ID:</span>
                        <span className="font-mono text-gray-900">#{selectedOrder.id}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600 font-medium">Status:</span>
                        {getStatusBadge(selectedOrder.status)}
                      </div>
                      <div className="flex justify-between">
                        <span className="text-gray-600 font-medium">Item:</span>
                        <span className="text-gray-900 font-medium">{selectedOrder.itemName}</span>
                      </div>
                      {/* {selectedOrder.manualRequired && ( */}
                        <div className="flex justify-between">
                          <span className="text-gray-600 font-medium">Manual Required:</span>
                          <span className="text-red-600 font-medium flex items-center">
                            <svg className="w-4 h-4 mr-1" fill="currentColor" viewBox="0 0 20 20">
                              <path fillRule="evenodd" d="M18 10a8 8 0 11-16 0 8 8 0 0116 0zm-7 4a1 1 0 11-2 0 1 1 0 012 0zm-1-9a1 1 0 00-1 1v4a1 1 0 102 0V6a1 1 0 00-1-1z" clipRule="evenodd" />
                            </svg>
                            {selectedOrder.manualRequired  ? 'Yes' : 'No'}
                          </span>
                        </div>
                      {/* )} */}
                    </div>
                  </div>
                  
                  <div>
                   {selectedOrder.cups && selectedOrder.cups.length > 0 && (
                    <div>
                      <h3 className="text-lg font-semibold text-gray-900 mb-4 flex items-center">
                        <svg className="w-5 h-5 mr-2 text-amber-500" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M20 7l-8-4-8 4m16 0l-8 4m8-4v10l-8 4m0-10L4 7m8 4v10M4 7v10l8 4" />
                        </svg>
                        Order Details
                      </h3>
                      <div className="space-y-4">
                        {selectedOrder.cups.map((cup, index) => (
                          <div key={index} className="bg-gradient-to-r from-amber-50 to-orange-50 border border-amber-200 rounded-lg p-4">
                            <div className="flex items-center justify-between mb-3">
                              <h4 className="font-semibold text-gray-900">Cup #{index + 1}</h4>
                              <span className="text-sm text-amber-600 font-medium bg-amber-100 px-2 py-1 rounded">
                                {cup.cup_size || cup.size || 'Standard'}
                              </span>
                            </div>
                            
                            <div className="grid grid-cols-2 gap-3 text-sm">
                              <div>
                                <span className="text-gray-600 font-medium">Drink:</span>
                                <p className="text-gray-900 mt-1">{cup.drink_type || cup.type || 'Unknown'}</p>
                              </div>
                              <div>
                                <span className="text-gray-600 font-medium">Size:</span>
                                <p className="text-gray-900 mt-1">{cup.cup_size || cup.size || 'Standard'}</p>
                              </div>
                            </div>
                            
                            {cup.addons && cup.addons.length > 0 && (
                              <div className="mt-3">
                                <span className="text-gray-600 font-medium text-sm">Add-ons:</span>
                                <div className="flex flex-wrap gap-1 mt-1">
                                  {cup.addons.map((addon, addonIndex) => (
                                    <span key={addonIndex} className="text-xs bg-gray-100 text-gray-700 px-2 py-1 rounded-full">
                                      {addon}
                                    </span>
                                  ))}
                                </div>
                              </div>
                            )}
                          </div>
                        ))}
                      </div>
                    </div>
                  )}
                  </div>
                </div>
                
                {/* Order Details */}
                <div className="space-y-6">

                  
                  <div>
                    <h3 className="text-lg font-semibold text-gray-900 mb-4 flex items-center">
                      <svg className="w-5 h-5 mr-2 text-green-500" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 8v4l3 3m6-3a9 9 0 11-18 0 9 9 0 0118 0z" />
                      </svg>
                      Timeline
                    </h3>
                    <div className="bg-gray-50 rounded-lg p-4 space-y-3">
                      <div className="flex justify-between">
                        <span className="text-gray-600 font-medium">Created:</span>
                        <span className="text-gray-900 font-mono text-sm">{selectedOrder.createdAt}</span>
                      </div>
                      {selectedOrder.startedAt !== 'N/A' && (
                        <div className="flex justify-between">
                          <span className="text-gray-600 font-medium">Started:</span>
                          <span className="text-gray-900 font-mono text-sm">{selectedOrder.startedAt}</span>
                        </div>
                      )}
                      {selectedOrder.completedAt !== 'N/A' && (
                        <div className="flex justify-between">
                          <span className="text-gray-600 font-medium">Completed:</span>
                          <span className="text-gray-900 font-mono text-sm">{selectedOrder.completedAt}</span>
                        </div>
                      )}
                      <div className="flex justify-between">
                        <span className="text-gray-600 font-medium">Last Updated:</span>
                        <span className="text-gray-900 font-mono text-sm">{selectedOrder.updatedAt}</span>
                      </div>
                    </div>
                  </div>
                </div>
              </div>
            </div>
            
            {/* Footer */}
            <div className="flex justify-end space-x-3 p-6 border-t border-gray-200 bg-gray-50">
              <button
                onClick={closeOrderDetails}
                className="px-4 py-2 text-gray-700 bg-white border border-gray-300 rounded-lg font-medium hover:bg-gray-50 transition-colors"
              >
                Close
              </button>
              
              {/* Action buttons based on order status */}
              {!['PROCESSING', 'STOPPING', 'COMPLETED', 'STOPPED', 'CANCELLED'].includes(selectedOrder.status) && (
                <>
                  {selectedOrder.status === 'QUEUED' && (
                    <button 
                      onClick={() => {
                        handleStartOrder(selectedOrder.id);
                        closeOrderDetails();
                      }}
                      className="px-4 py-2 bg-blue-600 hover:bg-blue-700 text-white rounded-lg font-medium transition-colors"
                    >
                      Start Order
                    </button>
                  )}
                  
                  {selectedOrder.status === 'HALTED' && (
                    <button 
                      onClick={() => {
                        handleResumeOrder(selectedOrder.id);
                        closeOrderDetails();
                      }}
                      className="px-4 py-2 bg-orange-600 hover:bg-orange-700 text-white rounded-lg font-medium transition-colors"
                    >
                      Resume Order
                    </button>
                  )}
                  
                  {selectedOrder.status === 'ERROR' && (
                    <button 
                      onClick={() => {
                        handleStartOrder(selectedOrder.id);
                        closeOrderDetails();
                      }}
                      className="px-4 py-2 bg-red-600 hover:bg-red-700 text-white rounded-lg font-medium transition-colors"
                    >
                      Retry Order
                    </button>
                  )}
                </>
              )}
            </div>
          </div>
        </div>
      )}
    </div>
  );
}

export default OrderQueue;
