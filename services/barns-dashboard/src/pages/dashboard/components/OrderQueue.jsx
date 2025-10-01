import React, { useState, useRef, useEffect } from 'react';
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

function SortableItem({ order, index, onStartOrder, onResumeOrder, onDeleteOrder, onViewDetails, isStarting, isDeleting, getStatusBadge }) {
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

  const disableDrag = index === 0 && order.status === 'PROCESSING';

  // Determine if buttons should be disabled
  const isDisabled = ['COMPLETED', 'CANCELLED'].includes(order.status);
  
  // Get status badge with colored dot
  const getStatusBadgeWithDot = (status) => {
    switch(status) {
      case 'PROCESSING':
        return (
          <span className="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-green-100 text-green-800">
            <div className="w-2 h-2 bg-green-600 rounded-full mr-2"></div>
            Processing
          </span>
        );
      case 'COMPLETED':
        return (
          <span className="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-gray-100 text-gray-800">
            <div className="w-2 h-2 bg-green-600 rounded-full mr-2"></div>
            Completed
          </span>
        );
      case 'QUEUED':
        return (
          <span className="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-gray-100 text-gray-800">
            <div className="w-2 h-2 bg-yellow-500 rounded-full mr-2"></div>
            Queued
          </span>
        );
      case 'CANCELLED':
        return (
          <span className="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-gray-100 text-gray-800">
            <div className="w-2 h-2 bg-red-600 rounded-full mr-2"></div>
            Canceled
          </span>
        );
      default:
        return (
          <span className="inline-flex items-center px-2.5 py-0.5 rounded-full text-xs font-medium bg-gray-100 text-gray-800">
            <div className="w-2 h-2 bg-blue-600 rounded-full mr-2"></div>
            {status}
          </span>
        );
    }
  };

  return (
    <div
      ref={setNodeRef}
      style={style}
      className={`mb-2 p-4 rounded-lg border border-gray-200 bg-white hover:shadow-sm transition-shadow duration-200`}
    >
      <div className="flex items-center justify-between">
        {/* Left side - ID and Status */}
        <div className="flex flex-col space-y-2">
          <span className="text-md text-bold text-gray-900">Order ID: {order.id}</span>
          {getStatusBadgeWithDot(order.status)}
        </div>
        
        {/* Right side - Action Buttons */}
        <div className="flex items-center space-x-2">
          {/* Start Button */}
          <button 
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

          {/* Details Button */}
          <button 
            onClick={() => onViewDetails(order)}
            disabled={isDisabled}
            className={`px-3 py-1.5 rounded text-xs font-medium border-2 transition-colors ${
              isDisabled 
                ? 'border-gray-300 text-gray-500 cursor-not-allowed'
                : 'border-green-600 text-green-600 hover:bg-green-50'
            }`}
            style={{
              borderWidth: '2px',
              borderStyle: 'solid',
              borderColor: isDisabled ? '#d1d5db' : '#059669'
            }}
            title={isDisabled ? "Action not available" : "View order details"}
          >
            Details
          </button>

          {/* Delete Button */}
          <button 
            onClick={() => onDeleteOrder(order.id)}
            disabled={isDisabled || isDeleting}
            className={`px-3 py-1.5 rounded text-xs font-medium border-2 transition-colors ${
              isDisabled 
                ? 'border-gray-300 text-gray-500 cursor-not-allowed'
                : isDeleting
                  ? 'border-red-300 text-red-500 cursor-not-allowed'
                  : 'border-red-600 text-red-600 hover:bg-red-50'
            }`}
            style={{
              borderWidth: '2px',
              borderStyle: 'solid',
              borderColor: isDisabled ? '#d1d5db' : isDeleting ? '#fca5a5' : '#dc2626'
            }}
            title={isDisabled ? "Action not available" : "Delete order"}
          >
            {isDeleting ? "Deleting..." : "Delete"}
          </button>
        </div>
      </div>
    </div>
  );
}

export default function OrderQueue({ connectionStatus }) {

  const { 
    orders, 
    recipes, 
    sendReorder, 
    startOrder, 
    resumeOrder, 
    deleteOrder, 
    createOrder, 
    fetchRecipes,
    isLoading, 
    errors, 
    clearError 
  } = useStore();
  
  const [selectedOrder, setSelectedOrder] = useState(null);
  const [filterStatus, setFilterStatus] = useState('ALL');
  const [searchTerm, setSearchTerm] = useState('');
  const [startingOrderId, setStartingOrderId] = useState(null);
  const [deletingOrderId, setDeletingOrderId] = useState(null);
  const [showNewOrder, setShowNewOrder] = useState(false);
  const [showOrderDetails, setShowOrderDetails] = useState(false);
  const [orderData, setOrderData] = useState({
    cups: [{ type: '', size: 'regular', addons: [] }]
  });
  const logsEndRef = useRef(null);

  // Fetch recipes when component loads
  useEffect(() => {
    if (recipes.length === 0) {
      fetchRecipes();
    }
  }, [fetchRecipes, recipes.length]);

  // Fallback recipes if API fails
  const fallbackRecipes = [
    { name: 'latte', display_name: 'Latte', steps: 4 },
    { name: 'americano', display_name: 'Americano', steps: 3 },
    { name: 'cappuccino', display_name: 'Cappuccino', steps: 5 },
    { name: 'espresso', display_name: 'Espresso', steps: 2 },
    { name: 'mocha', display_name: 'Mocha', steps: 6 }
  ];

  // Use API recipes if available, otherwise fallback to hardcoded ones
  const availableRecipes = recipes.length > 0 ? recipes : fallbackRecipes;
  
  const sizes = ['small', 'regular', 'large'];
  const addons = ['extra_shot', 'oat_milk', 'almond_milk', 'sugar_free', 'decaf'];

  const displayOrders = orders;

  const getStatusBadge = (status) => {
    switch(status) {
      case 'PROCESSING':
        return <span className="bg-yellow-100 text-yellow-800 text-xs font-medium px-2.5 py-0.5 rounded">🔄 Processing</span>;
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
    if (oldIndex === 0 && displayOrders[0].status === 'PROCESSING') return; // block moving processing
    
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

  const handleResumeOrder = async (orderId) => {
    console.log('🔄 handleResumeOrder called with orderId:', orderId);
    
    try {
      console.log('🔄 About to call resumeOrder from store...');
      const success = await resumeOrder(orderId);
      console.log('🔄 resumeOrder returned:', success);
      
      if (success) {
        console.log(`✅ Order ${orderId} resumed successfully`);
      } else {
        console.error(`❌ Failed to resume order ${orderId}`);
      }
    } catch (error) {
      console.error('🔄 Error in handleResumeOrder:', error);
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

  // const handleDeleteOrder = async (orderId) => {
  //   console.log('🗑️ handleDeleteOrder called with orderId:', orderId);
    
  //   // Find the order to get its status
  //   const order = displayOrders.find(o => o.id === orderId);
  //   const orderStatus = order?.status?.toUpperCase() || 'UNKNOWN';
    
  //   // Show different confirmation messages based on order status
  //   let confirmMessage;
  //   if (orderStatus === 'PROCESSING') {
  //     confirmMessage = `⚠️ WARNING: Order #${orderId} is currently being processed!\n\nDeleting this order will immediately stop all ongoing operations and may cause system issues.\n\nAre you absolutely sure you want to force delete this order?`;
  //   } else {
  //     confirmMessage = `Are you sure you want to delete order #${orderId}?\n\nThis action cannot be undone.`;
  //   }
    
  //   const confirmed = window.confirm(confirmMessage);
  //   if (!confirmed) {
  //     return;
  //   }
    
  //   setDeletingOrderId(orderId);
  //   try {
  //     console.log('🗑️ About to call deleteOrder from store...');
  //     const success = await deleteOrder(orderId);
  //     console.log('🗑️ deleteOrder returned:', success);
      
  //     if (success) {
  //       console.log(`✅ Order ${orderId} deleted successfully`);
  //     } else {
  //       console.error(`❌ Failed to delete order ${orderId}`);
  //     }
  //   } catch (error) {
  //     console.error('🗑️ Error in handleDeleteOrder:', error);
  //   } finally {
  //     setDeletingOrderId(null);
  //   }
  // };

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

  // New Order functions
  const handleSubmitNewOrder = async (e) => {
    e.preventDefault();
    
    // Validate that all cups have a drink type selected
    const invalidCups = orderData.cups.filter(cup => !cup.type || cup.type.trim() === '');
    if (invalidCups.length > 0) {
      Swal.fire({
        title: 'Missing Drink Type',
        text: 'Please select a drink type for all drinks before creating the order.',
        icon: 'warning',
        timer: 3000,
        timerProgressBar: true,
        showConfirmButton: false
      });
      return;
    }
    
    const success = await createOrder(orderData);
    if (success) {
      setOrderData({ cups: [{ type: '', size: 'regular', addons: [] }] });
      setShowNewOrder(false);

      Swal.fire({
        title: 'Order!',
        text: 'Order created successfully!',
        icon: 'success',
        timer: 3000, // 3 seconds = 3000ms
        timerProgressBar: true,
        showConfirmButton: false
      });
    } else {
      
        
          Swal.fire({
          title: 'Order!',
          text: 'Failed to create order!',
          icon: 'error',
          timer: 3000, // 3 seconds = 3000ms
          timerProgressBar: true,
          showConfirmButton: false
      });
    }
  };

  const addDrink = () => {
    setOrderData(prev => ({
      cups: [...prev.cups, { type: '', size: 'regular', addons: [] }]
    }));
  };

  const removeDrink = (index) => {
    if (orderData.cups.length > 1) {
      setOrderData(prev => ({
        cups: prev.cups.filter((_, i) => i !== index)
      }));
    }
  };

  const updateDrink = (index, field, value) => {
    setOrderData(prev => ({
      cups: prev.cups.map((cup, i) => 
        i === index ? { ...cup, [field]: value } : cup
      )
    }));
  };

  const toggleAddon = (drinkIndex, addon) => {
    setOrderData(prev => ({
      cups: prev.cups.map((cup, i) => {
        if (i === drinkIndex) {
          const currentAddons = cup.addons || [];
          const newAddons = currentAddons.includes(addon)
            ? currentAddons.filter(a => a !== addon)
            : [...currentAddons, addon];
          return { ...cup, addons: newAddons };
        }
        return cup;
      })
    }));
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
            {errors.recipes && showNewOrder && (
              <span className="ml-2 inline-flex items-center px-2 py-1 rounded-full text-xs font-medium bg-yellow-100 text-yellow-800">
                Recipes Error
              </span>
            )}
          </div>
          
          <h2
            onClick={() => setShowNewOrder(!showNewOrder)}
          className={`text-sm rounded font-small transition-colors duration-300  px-4 py-2 cursor-pointer
            ${
              showNewOrder
                ? 'bg-gray-100 text-white hover:bg-gray-200 button-sm'
                : 'barns-dark-bg text-white hover:barns-bg'
            }`}

            style={{'color':'white'}}
          >
            {showNewOrder ? 'View Orders' : 'New Order'}
          </h2>
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

      {/* Recipes Error display */}
      {errors.recipes && showNewOrder && (
        <div className="border-b border-yellow-200 bg-yellow-50 px-4 py-2 text-sm text-yellow-700 flex justify-between items-center flex-shrink-0">
          <div>
            <span className="font-medium">Recipes Error:</span> {errors.recipes}
            <p className="text-xs mt-1">Using fallback recipes. Please check the API connection.</p>
          </div>
          <button 
            onClick={() => fetchRecipes()}
            className="px-2 py-1 bg-yellow-100 hover:bg-yellow-200 text-yellow-800 rounded text-xs font-medium"
          >
            Retry
          </button>
        </div>
      )}
      
      {/* Scrollable content area */}
      <div className="flex-1 overflow-hidden">
        <div className="h-full overflow-y-auto p-4">
          {showNewOrder ? (
            /* New Order Form */
            <div className="space-y-6">
              {/* Custom Order Form */}
              <div>
                <div className="flex items-center justify-between mb-4">
                  <h3 className="text-lg font-semibold text-gray-900">New Order</h3>
                  <div className="flex items-center space-x-2">
                    <span className="text-sm text-gray-600">{orderData.cups.length} drink{orderData.cups.length !== 1 ? 's' : ''}</span>
                    {recipes.length === 0 && !errors.recipes && (
                      <span className="text-xs text-blue-600 flex items-center">
                        <svg className="animate-spin h-3 w-3 mr-1" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24">
                          <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
                          <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
                        </svg>
                        Loading recipes...
                      </span>
                    )}
                  </div>
                </div>
                
                <form onSubmit={handleSubmitNewOrder} className="space-y-4">
                  {/* Add/Remove Drinks and Submit */}
                  <div className="flex justify-between items-center sticky top-0 z-10  bg-gradient-to-b from-white/70 to-transparent backdrop-blur-sm ">
                    <h2
                      type="button"
                      onClick={addDrink}
                      className="px-4 py-2 barns-dark-bg text-white rounded-lg cursor-pointer text-sm"
                      style={{outline:'none', color:'white'}}
                    >
                      Add Drink
                    </h2>
                    
                    <button
                      type="submit"
                      disabled={isLoading || orderData.cups.some(cup => !cup.type || cup.type.trim() === '')}
                      className="px-6 py-2 bg-green-600 text-white rounded-lg hover:bg-green-700 disabled:opacity-50 disabled:cursor-not-allowed"
                    >
                      {isLoading ? 'Creating...' : `Create Order (${orderData.cups.length} drink${orderData.cups.length !== 1 ? 's' : ''})`}
                    </button>
                  </div>
                  {/* Multiple Drinks */}
                  {orderData.cups.map((cup, index) => (
                    <div key={index} className="border border-gray-200 rounded-lg p-4 bg-gray-50">
                      <div className="flex items-center justify-between mb-3">
                        <h4 className="text-sm font-medium text-gray-900">Drink {index + 1}</h4>
                        {orderData.cups.length > 1 && (
                          <button
                            type="button"
                            onClick={() => removeDrink(index)}
                            className="text-red-500 hover:text-red-700 text-sm"
                          >
                            Remove
                          </button>
                        )}
                      </div>
                      
                      <div className="grid grid-cols-1 md:grid-cols-2 gap-4 mb-3">
                        <div>
                          <label className="block text-sm font-medium text-gray-700 mb-1">Drink Type</label>
                          <select
                            value={cup.type}
                            onChange={(e) => updateDrink(index, 'type', e.target.value)}
                            className={`w-full px-3 py-2 border rounded-lg text-sm ${
                              !cup.type || cup.type.trim() === '' 
                                ? 'border-red-300 bg-red-50' 
                                : 'border-gray-300'
                            }`}
                            disabled={recipes.length === 0 && !errors.recipes}
                          >
                            {availableRecipes.length === 0 ? (
                              <option value="">Loading recipes...</option>
                            ) : (
                              <>
                                <option value="">Select drink type...</option>
                                {availableRecipes.map(recipe => (
                                  <option key={recipe.name} value={recipe.name}>
                                    {recipe.display_name}
                                  </option>
                                ))}
                              </>
                            )}
                          </select>
                          {(!cup.type || cup.type.trim() === '') && (
                            <p className="text-xs text-red-600 mt-1">Please select a drink type</p>
                          )}
                          {errors.recipes && (
                            <p className="text-xs text-yellow-600 mt-1">Using fallback recipes</p>
                          )}
                        </div>
                        
                        <div>
                          <label className="block text-sm font-medium text-gray-700 mb-1">Size</label>
                          <select
                            value={cup.size}
                            onChange={(e) => updateDrink(index, 'size', e.target.value)}
                            className="w-full px-3 py-2 border border-gray-300 rounded-lg text-sm"
                          >
                            {sizes.map(size => (
                              <option key={size} value={size}>
                                {size.charAt(0).toUpperCase() + size.slice(1)}
                              </option>
                            ))}
                          </select>
                        </div>
                      </div>

                      <div>
                        <label className="block text-sm font-medium text-gray-700 mb-2">Add-ons</label>
                        <div className="grid grid-cols-2 gap-2">
                          {addons.map(addon => (
                            <label key={addon} className="flex items-center">
                              <input
                                type="checkbox"
                                checked={cup.addons?.includes(addon) || false}
                                onChange={() => toggleAddon(index, addon)}
                                className="mr-2"
                              />
                              <span className="text-sm">{addon.replace('_', ' ')}</span>
                            </label>
                          ))}
                        </div>
                      </div>
                    </div>
                  ))}
                </form>
              </div>
            </div>
          ) : (
            /* Order Details View or Orders List */
            <>
              {selectedOrder && showOrderDetails ? (
                /* Order Details View */
                <div className="space-y-6">
                  {/* Header with Back Button */}
                  <div className="flex items-center space-x-4 mb-6">
                                          <button
                        onClick={closeOrderDetails}
                        className="border-0 outline-none bg-transparent p-0 m-0 no-outline-shadow no-outline-shadow"
                      >
                      <img src={backarrow} alt="Back" className="w-10 h-10" />
                    </button>
                    <div>
                      <h2 className="text-xl font-bold text-gray-900">Orders Details</h2>
                      <p className="text-sm text-gray-600">
                        ID: {selectedOrder.id} - Created {selectedOrder.createdAt}
                      </p>
                    </div>
                  </div>

                  {/* Order Items List */}
                  <div className="space-y-4">
                    {selectedOrder.cups && selectedOrder.cups.map((cup, index) => {
                      // Determine status and styling based on order status
                      let statusInfo = {
                        icon: null,
                        bgColor: '',
                        textColor: '',
                        status: ''
                      };

                      if (selectedOrder.status === 'COMPLETED') {
                        statusInfo = {
                          icon: (
                            <div className="w-8 h-8 bg-green-600 rounded flex items-center justify-center">
                              <svg className="w-5 h-5 text-white" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M5 13l4 4L19 7" />
                              </svg>
                            </div>
                          ),
                          bgColor: 'bg-green-50',
                          textColor: 'text-green-800',
                          status: `Completed ${selectedOrder.completedAt || 'N/A'}`
                        };
                      } else if (selectedOrder.status === 'PROCESSING') {
                        statusInfo = {
                          icon: (
                            <div className="w-8 h-8 bg-orange-400 rounded-full flex items-center justify-center">
                              <svg className="w-5 h-5 text-orange-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 4v5h.582m15.356 2A8.001 8.001 0 004.582 9m0 0H9m11 11v-5h-.581m0 0a8.003 8.003 0 01-15.357-2m15.357 2H15" />
                              </svg>
                            </div>
                          ),
                          bgColor: 'bg-orange-50',
                          textColor: 'text-orange-800',
                          status: 'Still Processing'
                        };
                      } else {
                        statusInfo = {
                          icon: (
                            <div className="w-8 h-8 bg-gray-300 rounded-full flex items-center justify-center">
                              <svg className="w-5 h-5 text-gray-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
                              </svg>
                            </div>
                          ),
                          bgColor: 'bg-gray-100',
                          textColor: 'text-gray-600',
                          status: 'Uncompleted'
                        };
                      }

                      return (
                        <div key={index} className={`${statusInfo.bgColor} rounded-lg p-4 shadow-sm border border-gray-200`}>
                          <div className="flex items-center space-x-4">
                            {statusInfo.icon}
                            <div className="flex-1">
                              <h3 className="font-medium text-gray-900 text-lg">
                                {cup.drink_type || cup.type || 'Unknown Item'}
                              </h3>
                              <p className={`text-sm ${statusInfo.textColor}`}>
                                {statusInfo.status}
                              </p>
                            </div>
                          </div>
                        </div>
                      );
                    })}
                  </div>
                </div>
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
                              onResumeOrder={handleResumeOrder}
                              onDeleteOrder={handleDeleteOrder}
                              onViewDetails={viewOrderDetails}
                              isStarting={startingOrderId === order.id}
                              isDeleting={deletingOrderId === order.id}
                              getStatusBadge={getStatusBadge}
                            />
                          ))}
                        </div>
                      </SortableContext>
                    </DndContext>
                  )}
                </>
              )}
            </>
          )}
        </div>
      </div>
    </div>
  );
}
