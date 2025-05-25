import React, { useState } from 'react';
import { DndContext, closestCenter } from '@dnd-kit/core';
import {
  arrayMove,
  SortableContext,
  useSortable,
  verticalListSortingStrategy
} from '@dnd-kit/sortable';
import { CSS } from '@dnd-kit/utilities';
import useStore from '../store';

function SortableItem({ order, index, onStartOrder, onResumeOrder, onViewDetails, isStarting, getStatusBadge }) {
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

  return (
    <div
      ref={setNodeRef}
      style={style}
      className={`mb-2 p-3 rounded border ${
        order.status === 'PROCESSING' ? 'bg-yellow-50 border-yellow-200' : 
        order.status === 'COMPLETED' ? 'bg-green-50 border-green-200' :
        order.status === 'HALTED' ? 'bg-orange-50 border-orange-200' :
        order.status === 'STOPPED' ? 'bg-red-50 border-red-200' :
        order.status === 'ERROR' ? 'bg-red-50 border-red-200' :
        order.status === 'CANCELLED' ? 'bg-gray-50 border-gray-200' :
        'bg-white border-gray-200'
      } hover:shadow-md transition-shadow duration-200`}
    >
      <div className="flex flex-col sm:flex-row sm:justify-between sm:items-center">
        {/* Draggable area - only the content area, not the buttons */}
        <div 
          {...attributes}
          {...listeners}
          className={`mb-2 sm:mb-0 flex-1 ${disableDrag ? 'cursor-default' : 'cursor-grab active:cursor-grabbing'}`}
        >
          <span className="text-sm font-medium text-gray-500">Order #{order.id}</span>
          <h3 className="font-semibold">{order.itemName}</h3>
          {order.manualRequired && (
            <div className="text-xs text-red-600 mt-1 flex items-center">
              <svg className="w-3 h-3 mr-1" fill="currentColor" viewBox="0 0 20 20" xmlns="http://www.w3.org/2000/svg">
                <path fillRule="evenodd" d="M18 10a8 8 0 11-16 0 8 8 0 0116 0zm-7-4a1 1 0 11-2 0 1 1 0 012 0zM9 9a1 1 0 000 2v3a1 1 0 001 1h1a1 1 0 100-2v-3a1 1 0 00-1-1H9z" clipRule="evenodd" />
              </svg>
              Manual step required
            </div>
          )}
        </div>
        
        {/* Button area - NOT draggable */}
        <div className="flex items-center space-x-2">
          {getStatusBadge(order.status)}
          
          {/* Action buttons */}
          <div className="flex space-x-1">
            {/* Debug: Log button condition */}
            {console.log(`🔍 Order ${order.id}: status="${order.status}", shouldShowButton:`, !['PROCESSING', 'COMPLETED', 'STOPPED', 'CANCELLED'].includes(order.status))}
            {!['PROCESSING', 'COMPLETED', 'STOPPED', 'CANCELLED'].includes(order.status) && (
              <>
                {/* Start Button for Queued Orders */}
                {order.status === 'QUEUED' && (
                  <button 
                    onClick={() => {
                      console.log('🔘 Start button clicked for order:', order.id);
                      onStartOrder(order.id);
                    }}
                    disabled={isStarting === order.id}
                    className={`text-xs px-2 py-1 rounded flex items-center ${
                      isStarting === order.id 
                        ? 'bg-blue-300 text-white cursor-not-allowed' 
                        : 'bg-blue-500 hover:bg-blue-600 text-white'
                    }`}
                    title={isStarting === order.id ? "Starting..." : "Start processing"}
                  >
                    {isStarting === order.id ? (
                      <>
                        <svg className="animate-spin h-3 w-3 mr-1" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24">
                          <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
                          <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
                        </svg>
                        Starting...
                      </>
                    ) : (
                      <>
                        <svg className="w-3 h-3 mr-1" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M14.828 14.828a4 4 0 01-5.656 0M9 10h1m4 0h1m-6 4h.01M19 10a9 9 0 11-18 0 9 9 0 0118 0z" />
                        </svg>
                        Start
                      </>
                    )}
                  </button>
                )}

                {/* Resume Button for Halted Orders */}
                {order.status === 'HALTED' && (
                  <button 
                    onClick={() => {
                      console.log('🔄 Resume button clicked for order:', order.id);
                      onResumeOrder && onResumeOrder(order.id);
                    }}
                    className="text-xs px-2 py-1 rounded flex items-center bg-orange-500 hover:bg-orange-600 text-white"
                    title="Resume halted order"
                  >
                    <svg className="w-3 h-3 mr-1" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                      <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M14.828 14.828a4 4 0 01-5.656 0M9 10h1m4 0h1m-6 4h.01M19 10a9 9 0 11-18 0 9 9 0 0118 0z" />
                    </svg>
                    Resume
                  </button>
                )}

                {/* Retry Button for Error Orders */}
                {order.status === 'ERROR' && (
                  <button 
                    onClick={() => {
                      console.log('🔄 Retry button clicked for order:', order.id);
                      onStartOrder(order.id);
                    }}
                    disabled={isStarting === order.id}
                    className={`text-xs px-2 py-1 rounded flex items-center ${
                      isStarting === order.id 
                        ? 'bg-red-300 text-white cursor-not-allowed' 
                        : 'bg-red-500 hover:bg-red-600 text-white'
                    }`}
                    title="Retry failed order"
                  >
                    <svg className="w-3 h-3 mr-1" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                      <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 4v5h.582m15.356 2A8.001 8.001 0 004.582 9m0 0V9a8 8 0 1115.356 2m-15.356-2H9" />
                    </svg>
                    Retry
                  </button>
                )}
              </>
            )}

            <button 
              onClick={() => onViewDetails(order)}
              className="text-xs px-2 py-1 bg-gray-100 hover:bg-gray-200 text-gray-700 rounded flex items-center"
              title="View order details"
            >
              <svg className="w-3 h-3 mr-1" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M15 12a3 3 0 11-6 0 3 3 0 016 0z" />
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M2.458 12C3.732 7.943 7.523 5 12 5c4.478 0 8.268 2.943 9.542 7-1.274 4.057-5.064 7-9.542 7-4.477 0-8.268-2.943-9.542-7z" />
              </svg>
              Details
            </button>
          </div>
        </div>
      </div>
    </div>
  );
}

export default function OrderQueue() {
  const { orders, sendReorder, startOrder, resumeOrder, isLoading, errors, clearError } = useStore();
  const [selectedOrder, setSelectedOrder] = useState(null);
  const [filterStatus, setFilterStatus] = useState('ALL');
  const [searchTerm, setSearchTerm] = useState('');
  const [startingOrderId, setStartingOrderId] = useState(null);

  // Debug: Log component render and orders data
  console.log('🚀 OrderQueue component rendering');
  console.log('🚀 Raw orders from store:', orders);
  console.log('🚀 Is loading:', isLoading);
  console.log('🚀 Errors:', errors);

  // Use real orders data (no more mock fallback)
  const displayOrders = orders;

  // Shared function to get status badge
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

  const viewOrderDetails = (order) => {
    setSelectedOrder(order);
  };

  const closeOrderDetails = () => {
    setSelectedOrder(null);
  };

  const retryFetchOrders = () => {
    clearError('orders');
    useStore.getState().fetchOrders();
  };

  // Map order data to display format
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
    <div className="bg-white rounded shadow">
      <div className="flex flex-col sm:flex-row sm:items-center justify-between p-4 border-b border-gray-200">
        <div className="flex items-center">
          <h2 className="text-xl font-bold mb-2 sm:mb-0">Order Queue</h2>
          {errors.orders && (
            <span className="ml-2 inline-flex items-center px-2 py-1 rounded-full text-xs font-medium bg-red-100 text-red-800">
              API Error
            </span>
          )}
        </div>
        
        <div className="flex flex-col sm:flex-row gap-4">
          <div className="flex-1">
            <input
              type="text"
              placeholder="Search orders..."
              value={searchTerm}
              onChange={(e) => setSearchTerm(e.target.value)}
              className="w-full px-3 py-2 border border-gray-300 rounded-lg focus:ring-2 focus:ring-green-500 focus:border-green-500"
            />
          </div>
          <div className="w-full sm:w-40">
            <select
              value={filterStatus}
              onChange={(e) => setFilterStatus(e.target.value)}
              className="w-full px-3 py-2 border border-gray-300 rounded-lg focus:ring-2 focus:ring-green-500 focus:border-green-500"
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
        </div>
      </div>
      
      {/* API Error display */}
      {errors.orders && (
        <div className="border-b border-red-200 bg-red-50 px-4 py-2 text-sm text-red-700 flex justify-between items-center">
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
      
      <div className="p-4">
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
              {filteredOrders.map((order, idx) => (
                <SortableItem 
                  key={order.id} 
                  order={order} 
                  index={idx} 
                  onStartOrder={handleStartOrder}
                  onResumeOrder={handleResumeOrder}
                  onViewDetails={viewOrderDetails}
                  isStarting={startingOrderId === order.id}
                  getStatusBadge={getStatusBadge}
                />
              ))}
            </SortableContext>
          </DndContext>
        )}
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
                  <h3 className="text-xl font-bold text-gray-900">Order #{selectedOrder.id} Details</h3>
                  <p className="text-sm text-gray-600">Complete order information and progress tracking</p>
                </div>
                {getStatusBadge(selectedOrder.status)}
              </div>
              <button
                onClick={() => setSelectedOrder(null)}
                className="text-gray-400 hover:text-gray-600 transition-colors"
              >
                <svg className="w-6 h-6" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
                </svg>
              </button>
            </div>

            <div className="p-6 space-y-6">
              {/* Order Summary */}
              <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
                <div className="bg-gray-50 p-4 rounded-lg">
                  <h4 className="font-semibold text-gray-700 mb-3 flex items-center">
                    <svg className="w-5 h-5 mr-2 text-blue-500" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                      <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 16h-1v-4h-1m1-4h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
                    </svg>
                    Order Information
                  </h4>
                  <div className="space-y-2 text-sm">
                    <div className="flex justify-between">
                      <span className="text-gray-500">Items:</span>
                      <span className="font-medium">{selectedOrder.itemName}</span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-500">Status:</span>
                      <span className="font-medium">{selectedOrder.status}</span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-500">Created:</span>
                      <span className="font-medium">{selectedOrder.createdAt}</span>
                    </div>
                    <div className="flex justify-between">
                      <span className="text-gray-500">Last Updated:</span>
                      <span className="font-medium">{selectedOrder.updatedAt}</span>
                    </div>
                    {selectedOrder.startedAt && selectedOrder.startedAt !== 'N/A' && (
                      <div className="flex justify-between">
                        <span className="text-gray-500">Started:</span>
                        <span className="font-medium">{selectedOrder.startedAt}</span>
                      </div>
                    )}
                    {selectedOrder.completedAt && selectedOrder.completedAt !== 'N/A' && (
                      <div className="flex justify-between">
                        <span className="text-gray-500">Completed:</span>
                        <span className="font-medium">{selectedOrder.completedAt}</span>
                      </div>
                    )}
                  </div>
                </div>

                {/* Task Summary */}
                {selectedOrder.task_summary && (
                  <div className="bg-blue-50 p-4 rounded-lg">
                    <h4 className="font-semibold text-gray-700 mb-3 flex items-center">
                      <svg className="w-5 h-5 mr-2 text-blue-500" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 5H7a2 2 0 00-2 2v10a2 2 0 002 2h8a2 2 0 002-2V7a2 2 0 00-2-2h-2M9 5a2 2 0 002 2h2a2 2 0 002-2M9 5a2 2 0 012-2h2a2 2 0 012 2" />
                      </svg>
                      Task Progress
                    </h4>
                    <div className="grid grid-cols-2 gap-3 text-sm">
                      <div className="flex justify-between">
                        <span className="text-gray-500">Total Tasks:</span>
                        <span className="font-medium">{selectedOrder.task_summary.total_tasks}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-green-600">Completed:</span>
                        <span className="font-medium text-green-600">{selectedOrder.task_summary.completed_tasks}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-blue-600">Running:</span>
                        <span className="font-medium text-blue-600">{selectedOrder.task_summary.running_tasks}</span>
                      </div>
                      <div className="flex justify-between">
                        <span className="text-red-600">Failed:</span>
                        <span className="font-medium text-red-600">{selectedOrder.task_summary.failed_tasks}</span>
                      </div>
                      {selectedOrder.task_summary.halted_tasks > 0 && (
                        <div className="flex justify-between col-span-2">
                          <span className="text-orange-600">Halted:</span>
                          <span className="font-medium text-orange-600">{selectedOrder.task_summary.halted_tasks}</span>
                        </div>
                      )}
                    </div>
                  </div>
                )}
              </div>

              {/* Scheduling Timeline */}
              {selectedOrder.timeline && selectedOrder.timeline.length > 0 && (
                <div className="bg-white border border-gray-200 rounded-lg p-4">
                  <h4 className="font-semibold text-gray-700 mb-4 flex items-center">
                    <svg className="w-5 h-5 mr-2 text-green-500" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                      <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 8v4l3 3m6-3a9 9 0 11-18 0 9 9 0 0118 0z" />
                    </svg>
                    Scheduling Timeline
                  </h4>
                  <div className="space-y-3 max-h-96 overflow-y-auto">
                    {selectedOrder.timeline.map((item, index) => (
                      <div key={index} className="flex items-start space-x-3">
                        <div className="flex-shrink-0 mt-1">
                          {item.status === 'completed' && (
                            <div className="w-4 h-4 bg-green-500 rounded-full flex items-center justify-center">
                              <svg className="w-2.5 h-2.5 text-white" fill="currentColor" viewBox="0 0 20 20">
                                <path fillRule="evenodd" d="M16.707 5.293a1 1 0 010 1.414l-8 8a1 1 0 01-1.414 0l-4-4a1 1 0 011.414-1.414L8 12.586l7.293-7.293a1 1 0 011.414 0z" clipRule="evenodd" />
                              </svg>
                            </div>
                          )}
                          {item.status === 'in_progress' && (
                            <div className="w-4 h-4 bg-blue-500 rounded-full animate-pulse"></div>
                          )}
                          {item.status === 'error' && (
                            <div className="w-4 h-4 bg-red-500 rounded-full flex items-center justify-center">
                              <svg className="w-2.5 h-2.5 text-white" fill="currentColor" viewBox="0 0 20 20">
                                <path fillRule="evenodd" d="M4.293 4.293a1 1 0 011.414 0L10 8.586l4.293-4.293a1 1 0 111.414 1.414L11.414 10l4.293 4.293a1 1 0 01-1.414 1.414L10 11.414l-4.293 4.293a1 1 0 01-1.414-1.414L8.586 10 4.293 5.707a1 1 0 010-1.414z" clipRule="evenodd" />
                              </svg>
                            </div>
                          )}
                          {item.status === 'pending' && (
                            <div className="w-4 h-4 bg-gray-300 rounded-full"></div>
                          )}
                        </div>
                        <div className="flex-1 min-w-0">
                          <div className="flex items-center justify-between">
                            <p className={`text-sm font-medium ${
                              item.step.startsWith('  └─') ? 'text-gray-600 ml-4' : 'text-gray-900'
                            }`}>
                              {item.step}
                            </p>
                            {item.timestamp && (
                              <span className="text-xs text-gray-500">
                                {new Date(item.timestamp).toLocaleTimeString()}
                              </span>
                            )}
                          </div>
                          {item.description && (
                            <p className={`text-xs text-gray-500 mt-1 ${
                              item.step.startsWith('  └─') ? 'ml-4' : ''
                            }`}>
                              {item.description}
                            </p>
                          )}
                        </div>
                      </div>
                    ))}
                  </div>
                </div>
              )}

              {/* Task Details */}
              {selectedOrder.tasks && selectedOrder.tasks.length > 0 && (
                <div className="bg-white border border-gray-200 rounded-lg p-4">
                  <h4 className="font-semibold text-gray-700 mb-4 flex items-center">
                    <svg className="w-5 h-5 mr-2 text-purple-500" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                      <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 11H5m14 0a2 2 0 012 2v6a2 2 0 01-2 2H5a2 2 0 01-2-2v-6a2 2 0 012-2m14 0V9a2 2 0 00-2-2M5 11V9a2 2 0 012-2m0 0V5a2 2 0 012-2h6a2 2 0 012 2v2M7 7h10" />
                    </svg>
                    Detailed Task Information
                  </h4>
                  <div className="space-y-3 max-h-64 overflow-y-auto">
                    {selectedOrder.tasks.map((task, index) => (
                      <div key={index} className="border border-gray-100 rounded-lg p-3 bg-gray-50">
                        <div className="flex items-center justify-between mb-2">
                          <h5 className="font-medium text-gray-800">
                            {task.function_name} (Arm {task.arm_id})
                          </h5>
                          <span className={`px-2 py-1 text-xs rounded font-medium ${
                            task.status === 'completed' ? 'bg-green-100 text-green-800' :
                            task.status === 'running' ? 'bg-blue-100 text-blue-800' :
                            task.status === 'failed' ? 'bg-red-100 text-red-800' :
                            task.status === 'halted' ? 'bg-orange-100 text-orange-800' :
                            'bg-gray-100 text-gray-800'
                          }`}>
                            {task.status}
                          </span>
                        </div>
                        {task.error_message && (
                          <p className="text-sm text-red-600 mb-2">{task.error_message}</p>
                        )}
                        <div className="text-xs text-gray-500 space-y-1">
                          {task.started_at && (
                            <p>Started: {new Date(task.started_at).toLocaleString()}</p>
                          )}
                          {task.completed_at && (
                            <p>Completed: {new Date(task.completed_at).toLocaleString()}</p>
                          )}
                        </div>
                      </div>
                    ))}
                  </div>
                </div>
              )}

              {/* Close Button */}
              <div className="flex justify-end pt-4 border-t border-gray-200">
                <button
                  onClick={() => setSelectedOrder(null)}
                  className="bg-gray-500 hover:bg-gray-600 text-white px-6 py-2 rounded-lg transition-colors"
                >
                  Close
                </button>
              </div>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}
