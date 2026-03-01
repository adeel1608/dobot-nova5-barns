/**
 * CompactItemCustomization
 * Touch-screen variant of ItemCustomization for the POS compact layout.
 * All interactive targets are 36-44px. Temperature and espresso options
 * fill the full row width. Steppers use 36px buttons.
 */

import React, { useState, useEffect } from 'react';
import {
  TEMPERATURE_OPTIONS,
  DEFAULT_TEMPERATURE,
  INGREDIENT_DENSITIES,
  MAX_MILK_SUBSTITUTION_PERCENT,
  CUP_VOLUMES,
  ESPRESSO_SHOT_WEIGHTS,
  isIcedCup
} from '../../../../constants/cupCapacityConfig';
import {
  calculateAdjustedMilk,
  calculateRecipeVolume,
  getIngredientWeight
} from '../../utils/cupCapacityCalculator';

/* ─── Section label ─────────────────────────────────────────────── */
function SectionLabel({ children }) {
  return (
    <span className="block text-[10px] font-bold text-gray-400 uppercase tracking-widest mb-1.5">
      {children}
    </span>
  );
}

/* ─── Amount stepper — 36px buttons for touch ───────────────────── */
function AmountStepper({ label, value, unit, levelLabel, onDecrease, onIncrease, decreaseDisabled, increaseDisabled, accentClass }) {
  return (
    <div className="flex items-center justify-between py-1">
      <SectionLabel>{label}</SectionLabel>
      <div className="flex items-center gap-2">
        <button
          type="button"
          onClick={onDecrease}
          disabled={decreaseDisabled}
          className={`w-9 h-9 rounded-xl flex items-center justify-center text-base font-bold transition-all active:scale-90 disabled:opacity-30 disabled:cursor-not-allowed ${accentClass}`}
          style={{ border: '2px solid currentColor' }}
        >
          −
        </button>
        <div className="flex flex-col items-center min-w-[56px]">
          <span className="text-sm font-bold text-gray-800 leading-none">{value}{unit}</span>
          <span className="text-[9px] text-gray-400 mt-0.5 uppercase tracking-wide leading-none">{levelLabel}</span>
        </div>
        <button
          type="button"
          onClick={onIncrease}
          disabled={increaseDisabled}
          className={`w-9 h-9 rounded-xl flex items-center justify-center text-base font-bold transition-all active:scale-90 disabled:opacity-30 disabled:cursor-not-allowed ${accentClass}`}
          style={{ border: '2px solid currentColor' }}
        >
          +
        </button>
      </div>
    </div>
  );
}

/* ─── Capacity bar ───────────────────────────────────────────────── */
function CapacityBar({ segments, freePercent, isExceeded, warning }) {
  return (
    <div className="mb-3">
      <div className="flex justify-between items-center mb-1">
        <SectionLabel>Cup Capacity</SectionLabel>
        {isExceeded && <span className="text-[9px] font-bold text-red-500 uppercase">Over limit</span>}
      </div>
      <div className="h-3 flex rounded-full overflow-hidden border border-gray-200 bg-gray-100">
        {segments.map((seg, i) =>
          seg.percent > 0 ? (
            <div key={i} className={seg.className} style={{ width: `${seg.percent}%` }} title={seg.title} />
          ) : null
        )}
        {freePercent > 0 && (
          <div
            style={{
              width: `${freePercent}%`,
              background: 'repeating-linear-gradient(45deg,#f9fafb,#f9fafb 4px,#f3f4f6 4px,#f3f4f6 8px)'
            }}
            title={`Free: ${freePercent.toFixed(0)}%`}
          />
        )}
      </div>
      {warning && (
        <p className={`text-[10px] font-semibold mt-1 ${isExceeded ? 'text-red-500' : 'text-amber-500'}`}>
          {warning}
        </p>
      )}
    </div>
  );
}

/* ─── Full-width pill group (temperature, espresso, etc.) ────────── */
function PillGroup({ options, selected, onSelect, getLabel, getId, getBadge, getWouldExceed }) {
  return (
    <div className="flex gap-2 flex-wrap">
      {options.map((opt) => {
        const id = getId(opt);
        const label = getLabel(opt);
        const badge = getBadge ? getBadge(opt) : null;
        const wouldExceed = getWouldExceed ? getWouldExceed(opt) : false;
        const isActive = selected === id;
        return (
          <button
            key={id}
            type="button"
            onClick={() => onSelect(id)}
            style={{ fontSize: '13px', border: isActive ? '2px solid #16a34a' : wouldExceed ? '2px solid #fbbf24' : '2px solid #9ca3af' }}
            className={`flex-1 min-h-[36px] whitespace-nowrap font-semibold rounded-xl px-2 py-1 transition-all duration-120 active:scale-95 flex items-center justify-center ${isActive
              ? 'bg-green-600 text-white shadow-sm'
              : wouldExceed
                ? 'bg-amber-50 text-amber-700'
                : 'bg-white text-gray-700 hover:text-green-700'
              }`}
          >
            {label}
          </button>
        );
      })}
    </div>
  );
}

/* ─── Divider ────────────────────────────────────────────────────── */
function Divider() {
  return <div className="border-t border-gray-100 my-2" />;
}

/* ─── Main component ─────────────────────────────────────────────── */
export default function CompactItemCustomization({
  item,
  itemId,
  addKitchenNote,
  updateKitchenNote,
  removeKitchenNote,
  addIngredientReplacement,
  addIngredientAddon,
  removeIngredientModification,
  getIngredientNameById,
  getIngredientDetailsById,
  ingredientsByCategory,
  updateCartItem
}) {
  const hiddenCategories = ['premixes', 'sachets', 'cups', 'position', 'temperature', 'ice', 'water'];
  const replaceableCategories = ['espresso', 'milk'];

  /* ── State ── */
  const [selectedTemperature, setSelectedTemperature] = useState(DEFAULT_TEMPERATURE);

  const [iceAmount, setIceAmount] = useState(() => {
    const ing = item.selectedMenuItem?.default_ingredients?.find(i => i.category === 'ice');
    return ing ? Math.min((ing.unit_amount || 0) * (ing.quantity || 1), 300) : 0;
  });

  const [milkAmount, setMilkAmount] = useState(() => {
    const ing = item.selectedMenuItem?.default_ingredients?.find(i => i.category === 'milk');
    return ing ? Math.min((ing.unit_amount || 0) * (ing.quantity || 1), 500) : 0;
  });

  const [waterAmount, setWaterAmount] = useState(() => {
    const ing = item.selectedMenuItem?.default_ingredients?.find(i => i.category === 'water');
    return ing ? Math.min((ing.unit_amount || 0) * (ing.quantity || 1), 500) : 0;
  });

  const [capacityInfo, setCapacityInfo] = useState(null);
  const [showNotes, setShowNotes] = useState(false);

  /* ── Sync amounts on item change ── */
  useEffect(() => {
    const ing = item.selectedMenuItem?.default_ingredients?.find(i => i.category === 'ice');
    setIceAmount(ing ? Math.min((ing.unit_amount || 0) * (ing.quantity || 1), 300) : 0);
  }, [item.selectedMenuItem]);

  useEffect(() => {
    const ing = item.selectedMenuItem?.default_ingredients?.find(i => i.category === 'milk');
    setMilkAmount(ing ? Math.min((ing.unit_amount || 0) * (ing.quantity || 1), 500) : 0);
  }, [item.selectedMenuItem]);

  useEffect(() => {
    const ing = item.selectedMenuItem?.default_ingredients?.find(i => i.category === 'water');
    setWaterAmount(ing ? Math.min((ing.unit_amount || 0) * (ing.quantity || 1), 500) : 0);
  }, [item.selectedMenuItem]);

  /* ── Initialisation refs ── */
  const tempInit = React.useRef(false);
  const iceInit = React.useRef(false);
  const milkInit = React.useRef(false);
  const waterInit = React.useRef(false);

  useEffect(() => {
    if (!item.selectedMenuItem || tempInit.current) return;
    tempInit.current = true;
    if (!(item.kitchen_notes || []).find(n => n.type?.toLowerCase().includes('temperature'))) {
      updateCartItem(itemId, { kitchen_notes: [...(item.kitchen_notes || []), { type: 'Temperature', detail: DEFAULT_TEMPERATURE, qty: 0 }] });
    }
  }, [item.selectedMenuItem, item.kitchen_notes, itemId, updateCartItem]);

  useEffect(() => {
    if (!item.selectedMenuItem || iceInit.current) return;
    const ing = item.selectedMenuItem.default_ingredients?.find(i => i.category === 'ice');
    iceInit.current = true;
    if (!ing || (item.item_ingredients || []).find(m => m.isIceModification)) return;
    const def = Math.min((ing.unit_amount || 0) * (ing.quantity || 1), 300);
    updateCartItem(itemId, { item_ingredients: [...(item.item_ingredients || []), { itemId: ing.ingredient_id, initialItemId: ing.ingredient_id, category: 'ice', isModified: true, isAddon: false, isIceModification: true, iceAmountGrams: def, qty: def }] });
  }, [item.selectedMenuItem, item.item_ingredients, itemId, updateCartItem]);

  useEffect(() => {
    if (!item.selectedMenuItem || milkInit.current) return;
    const ing = item.selectedMenuItem.default_ingredients?.find(i => i.category === 'milk');
    milkInit.current = true;
    if (!ing || (item.item_ingredients || []).find(m => m.isMilkAmountModification)) return;
    const def = Math.min((ing.unit_amount || 0) * (ing.quantity || 1), 500);
    updateCartItem(itemId, { item_ingredients: [...(item.item_ingredients || []), { itemId: ing.ingredient_id, initialItemId: ing.ingredient_id, category: 'milk', isModified: true, isAddon: false, isMilkAmountModification: true, isAmountModification: true, amountGrams: def, qty: def }] });
  }, [item.selectedMenuItem, item.item_ingredients, itemId, updateCartItem]);

  useEffect(() => {
    if (!item.selectedMenuItem || waterInit.current) return;
    const ing = item.selectedMenuItem.default_ingredients?.find(i => i.category === 'water');
    waterInit.current = true;
    if (!ing || (item.item_ingredients || []).find(m => m.isWaterAmountModification)) return;
    const def = Math.min((ing.unit_amount || 0) * (ing.quantity || 1), 500);
    updateCartItem(itemId, { item_ingredients: [...(item.item_ingredients || []), { itemId: ing.ingredient_id, initialItemId: ing.ingredient_id, category: 'water', isModified: true, isAddon: false, isWaterAmountModification: true, isAmountModification: true, amountGrams: def, qty: def }] });
  }, [item.selectedMenuItem, item.item_ingredients, itemId, updateCartItem]);

  /* ── Espresso type ── */
  const getCurrentEspressoType = () => {
    const def = item.selectedMenuItem?.default_ingredients?.find(i => i.category === 'espresso');
    if (!def) return 'double_shot';
    const rep = (item.item_ingredients || []).find(m => !m.isAddon && m.isModified && String(m.initialItemId) === String(def.ingredient_id));
    if (rep) {
      const d = getIngredientDetailsById(rep.itemId);
      const n = (d?.type || d?.name || rep.itemId || '').toString().toLowerCase().replace(/\s+/g, '_');
      if (n.includes('single')) return 'single_shot';
      if (n.includes('double')) return 'double_shot';
      if (n.includes('triple') || n.includes('tripple')) return 'tripple_shot';
      return n || def.type || 'double_shot';
    }
    return def.type || 'double_shot';
  };
  const currentEspressoType = getCurrentEspressoType();

  /* ── Capacity calculation ── */
  useEffect(() => {
    if (!item.selectedMenuItem) return;
    const milkIng = item.selectedMenuItem.default_ingredients?.find(i => i.category === 'milk');
    let baseMilkAmount = 0;
    if (milkIng) {
      baseMilkAmount = (milkIng.base_units || '').toLowerCase() === 'ml' ? milkAmount : milkAmount / INGREDIENT_DENSITIES.milk;
    }
    const cupIng = item.selectedMenuItem.default_ingredients?.find(i => i.category === 'cups');
    const cupSize = cupIng?.type || 'H9';
    const skipCats = ['milk', 'espresso', 'ice', 'cups', 'position', 'temperature'];
    let recipeVolume = 0;
    for (const ing of (item.selectedMenuItem.default_ingredients || [])) {
      if (skipCats.includes((ing.category || '').toLowerCase())) continue;
      const units = (ing.base_units || '').toLowerCase();
      recipeVolume += units === 'ml'
        ? (ing.unit_amount || 0) * (ing.quantity || 1)
        : ((ing.unit_amount || 0) * (ing.quantity || 1)) / (INGREDIENT_DENSITIES[(ing.category || '').toLowerCase()] || 1);
    }
    const hasEspresso = item.selectedMenuItem.default_ingredients?.some(i => i.category === 'espresso');
    if (hasEspresso) {
      const key = (currentEspressoType || 'double_shot').toLowerCase().replace(/\s+/g, '_');
      recipeVolume += (ESPRESSO_SHOT_WEIGHTS[key] || ESPRESSO_SHOT_WEIGHTS.double_shot) / INGREDIENT_DENSITIES.espresso;
    }
    if (iceAmount > 0) recipeVolume += iceAmount / INGREDIENT_DENSITIES.ice;

    const addons = (item.item_ingredients || [])
      .filter(m => m.isAddon)
      .map(m => {
        const d = getIngredientDetailsById(m.itemId);
        return { category: d?.category || 'extras', quantity: m.qty || 1, weight: getIngredientWeight(d?.category || 'extras', m.qty || 1) };
      })
      .filter(a => a.category !== 'ice');

    const cupVolume = CUP_VOLUMES[cupSize] || 266;
    let result;
    if (baseMilkAmount > 0) {
      result = calculateAdjustedMilk(cupSize, baseMilkAmount, selectedTemperature, addons, recipeVolume);
      result.actualFreeSpace = cupVolume - (result.adjustedMilkAmount + result.foamReductionAmount + recipeVolume + (result.totalAddonVolume || 0));
    } else {
      let addonVol = 0;
      const addonDetails = addons.map(a => { const v = a.weight / (INGREDIENT_DENSITIES[a.category] || 1); addonVol += v; return { ...a, volume: v }; });
      const free = cupVolume - recipeVolume - addonVol;
      result = { baseMilkAmount: 0, foamReducedMilk: 0, foamReductionAmount: 0, foamReductionPercent: 0, freeSpaceRemaining: free, actualFreeSpace: free, milkSubstitutionAmount: 0, milkSubstitutionPercent: 0, adjustedMilkAmount: 0, canAddMore: free > 0, remainingSubstitutionCapacity: 0, remainingSubstitutionPercent: 0, addonDetails, totalAddonVolume: addonVol, temperature: selectedTemperature };
    }
    setCapacityInfo(result);
    const exceeded = result.milkSubstitutionPercent > MAX_MILK_SUBSTITUTION_PERCENT || result.actualFreeSpace < 0;
    if (item.capacityExceeded !== exceeded) updateCartItem(itemId, { capacityExceeded: exceeded });
  }, [item, selectedTemperature, iceAmount, milkAmount, waterAmount, currentEspressoType, getIngredientDetailsById, itemId, updateCartItem]);

  /* ── Handlers ── */
  const handleTemperatureChange = (newTemp) => {
    const milkIng = item.selectedMenuItem?.default_ingredients?.find(i => i.category === 'milk');
    if (!milkIng) { setSelectedTemperature(newTemp); return; }
    const hasAddons = (item.item_ingredients || []).some(m => m.isAddon);
    if (hasAddons) {
      const w = (milkIng.unit_amount || 0) * (milkIng.quantity || 1);
      const base = (milkIng.base_units || '').toLowerCase() === 'ml' ? w : w / INGREDIENT_DENSITIES.milk;
      const cupIng = item.selectedMenuItem.default_ingredients?.find(i => i.category === 'cups');
      const sim = calculateAdjustedMilk(cupIng?.type || 'H9', base, newTemp,
        (item.item_ingredients || []).filter(m => m.isAddon).map(m => { const d = getIngredientDetailsById(m.itemId); return { category: d?.category || 'extras', quantity: m.qty || 1, weight: getIngredientWeight(d?.category || 'extras', m.qty || 1) }; }).filter(a => a.category !== 'ice'),
        calculateRecipeVolume(item.selectedMenuItem.default_ingredients || []));
      if (sim.milkSubstitutionPercent > 30) return;
    }
    setSelectedTemperature(newTemp);
    const idx = item.kitchen_notes.findIndex(n => n.type.toLowerCase().includes('temperature'));
    if (idx >= 0) updateKitchenNote(itemId, idx, 'detail', newTemp);
    else updateCartItem(itemId, { kitchen_notes: [...(item.kitchen_notes || []), { type: 'Temperature', detail: newTemp, qty: 0 }] });
  };

  const handleIceAdjustment = (change) => {
    let n = Math.max(0, Math.min(300, iceAmount + change));
    if (change > 0 && capacityInfo?.freeSpaceRemaining <= 0) return;
    setIceAmount(n);
    const ing = item.selectedMenuItem?.default_ingredients?.find(i => i.category === 'ice');
    if (!ing) return;
    const idx = (item.item_ingredients || []).findIndex(m => m.category === 'ice' && m.isIceModification);
    if (idx >= 0) { const arr = [...(item.item_ingredients || [])]; arr[idx] = { ...arr[idx], iceAmountGrams: n, qty: n }; updateCartItem(itemId, { item_ingredients: arr }); }
    else updateCartItem(itemId, { item_ingredients: [...(item.item_ingredients || []), { itemId: ing.ingredient_id, initialItemId: ing.ingredient_id, category: 'ice', isModified: true, isAddon: false, isIceModification: true, iceAmountGrams: n, qty: n }] });
  };

  const handleMilkAdjustment = (change) => {
    let n = Math.max(0, Math.min(500, milkAmount + change));
    if (change > 0 && capacityInfo?.freeSpaceRemaining <= 0) return;
    setMilkAmount(n);
    const ing = item.selectedMenuItem?.default_ingredients?.find(i => i.category === 'milk');
    if (!ing) return;
    const grams = (ing.base_units || '').toLowerCase() === 'ml' ? n * INGREDIENT_DENSITIES.milk : n;
    const idx = (item.item_ingredients || []).findIndex(m => m.category === 'milk' && m.isMilkAmountModification);
    if (idx >= 0) { const arr = [...(item.item_ingredients || [])]; arr[idx] = { ...arr[idx], amountGrams: grams, qty: grams }; updateCartItem(itemId, { item_ingredients: arr }); }
    else updateCartItem(itemId, { item_ingredients: [...(item.item_ingredients || []), { itemId: ing.ingredient_id, initialItemId: ing.ingredient_id, category: 'milk', isModified: true, isAddon: false, isMilkAmountModification: true, isAmountModification: true, amountGrams: grams, qty: grams }] });
  };

  const handleWaterAdjustment = (change) => {
    let n = Math.max(0, Math.min(500, waterAmount + change));
    if (change > 0 && capacityInfo?.freeSpaceRemaining <= 0) return;
    setWaterAmount(n);
    const ing = item.selectedMenuItem?.default_ingredients?.find(i => i.category === 'water');
    if (!ing) return;
    const grams = (ing.base_units || '').toLowerCase() === 'ml' ? n * (INGREDIENT_DENSITIES.water || 1) : n;
    const idx = (item.item_ingredients || []).findIndex(m => m.category === 'water' && m.isWaterAmountModification);
    if (idx >= 0) { const arr = [...(item.item_ingredients || [])]; arr[idx] = { ...arr[idx], amountGrams: grams, qty: grams }; updateCartItem(itemId, { item_ingredients: arr }); }
    else updateCartItem(itemId, { item_ingredients: [...(item.item_ingredients || []), { itemId: ing.ingredient_id, initialItemId: ing.ingredient_id, category: 'water', isModified: true, isAddon: false, isWaterAmountModification: true, isAmountModification: true, amountGrams: grams, qty: grams }] });
  };

  const getCurrentSelection = (category) => {
    const def = item.selectedMenuItem?.default_ingredients?.find(d => d.category === category);
    if (!def) return null;
    const rep = (item.item_ingredients || []).find(m => !m.isAddon && m.isModified && String(m.initialItemId) === String(def.ingredient_id));
    return rep || { itemId: def.ingredient_id, isDefault: true };
  };

  const handleReplaceableSelection = (category, newId) => {
    const def = item.selectedMenuItem?.default_ingredients?.find(d => d.category === category);
    if (!def) return;
    const idx = (item.item_ingredients || []).findIndex(m => !m.isAddon && m.isModified && String(m.initialItemId) === String(def.ingredient_id));
    if (idx >= 0) removeIngredientModification(itemId, idx);
    if (newId !== def.ingredient_id) addIngredientReplacement(itemId, def.ingredient_id, newId, 1, category);
  };

  const getAddonsForCategory = (category) =>
    (item.item_ingredients || []).map((m, idx) => ({ ...m, originalIndex: idx }))
      .filter(m => { if (!m.isAddon) return false; const d = getIngredientDetailsById(m.itemId); return d && d.category === category; });

  const updateAddonQuantity = (modIndex, newQty) => {
    if (newQty < 1) { removeIngredientModification(itemId, modIndex); return; }
    if (newQty > (item.item_ingredients || [])[modIndex]?.qty && capacityInfo && !capacityInfo.canAddMore) return;
    const arr = [...(item.item_ingredients || [])]; arr[modIndex] = { ...arr[modIndex], qty: newQty }; updateCartItem(itemId, { item_ingredients: arr });
  };

  const canAddMoreAddons = () => !capacityInfo || capacityInfo.canAddMore;

  const wouldTemperatureExceedCapacity = (temp) => {
    if (!capacityInfo || temp === selectedTemperature) return false;
    const milkIng = item.selectedMenuItem?.default_ingredients?.find(i => i.category === 'milk');
    if (!milkIng || !(item.item_ingredients || []).some(m => m.isAddon)) return false;
    const w = (milkIng.unit_amount || 0) * (milkIng.quantity || 1);
    const base = (milkIng.base_units || '').toLowerCase() === 'ml' ? w : w / INGREDIENT_DENSITIES.milk;
    const cupIng = item.selectedMenuItem.default_ingredients?.find(i => i.category === 'cups');
    const sim = calculateAdjustedMilk(cupIng?.type || 'H9', base, temp,
      (item.item_ingredients || []).filter(m => m.isAddon).map(m => { const d = getIngredientDetailsById(m.itemId); return { category: d?.category || 'extras', quantity: m.qty || 1, weight: getIngredientWeight(d?.category || 'extras', m.qty || 1) }; }).filter(a => a.category !== 'ice'),
      calculateRecipeVolume(item.selectedMenuItem.default_ingredients || []));
    return sim.milkSubstitutionPercent > 30;
  };

  /* ── Level label helper ── */
  const levelLabel = (amount, max) => {
    const def = (() => { const ing = item.selectedMenuItem?.default_ingredients?.find(i => i.category === (max === 300 ? 'ice' : max === 500 ? 'milk' : 'water')); return ing ? Math.min((ing.unit_amount || 0) * (ing.quantity || 1), max) : max * 0.5; })();
    if (amount === 0) return 'None';
    if (amount < def * 0.7) return 'Light';
    if (amount > def * 1.3) return 'Extra';
    return 'Normal';
  };

  /* ── Bar segments ── */
  const buildBarSegments = () => {
    if (!capacityInfo || !item.selectedMenuItem) return { segments: [], freePercent: 100, isExceeded: false, warning: null };
    const cupIng = item.selectedMenuItem.default_ingredients?.find(i => i.category === 'cups');
    const cupSize = cupIng?.type || 'H9';
    const cupVolume = CUP_VOLUMES[cupSize] || 266;
    const isIced = isIcedCup(cupSize);
    const milkIngBar = item.selectedMenuItem.default_ingredients?.find(i => i.category === 'milk');
    const milkRawVol = (milkIngBar?.base_units || '').toLowerCase() === 'ml' ? milkAmount : milkAmount / INGREDIENT_DENSITIES.milk;
    const tempFoamPct = isIced ? 0 : (TEMPERATURE_OPTIONS.find(t => t.id === selectedTemperature)?.foamPercent ?? 10);
    const foamVol = milkRawVol * (tempFoamPct / 100);
    const milkVol = milkRawVol - foamVol;
    let espressoVol = 0;
    const defEsp = item.selectedMenuItem.default_ingredients?.find(i => i.category === 'espresso');
    if (defEsp) { const key = (currentEspressoType || 'double_shot').toLowerCase().replace(/\s+/g, '_'); espressoVol = (ESPRESSO_SHOT_WEIGHTS[key] || ESPRESSO_SHOT_WEIGHTS.double_shot) / INGREDIENT_DENSITIES.espresso; }
    const iceVol = iceAmount > 0 ? iceAmount / INGREDIENT_DENSITIES.ice : 0;
    const otherVol = (capacityInfo.addonDetails || []).reduce((s, a) => s + a.volume, 0);
    const freeVol = Math.max(0, cupVolume - milkVol - foamVol - espressoVol - iceVol - otherVol);
    const pct = v => (v / cupVolume) * 100;
    const isExceeded = capacityInfo.milkSubstitutionPercent > MAX_MILK_SUBSTITUTION_PERCENT || capacityInfo.actualFreeSpace < 0;
    const segments = [
      { percent: pct(milkVol), className: 'bg-blue-200', title: `Milk: ${milkVol.toFixed(0)}ml` },
      { percent: pct(foamVol), className: 'bg-yellow-200', title: `Foam: ${foamVol.toFixed(0)}ml` },
      { percent: pct(espressoVol), className: 'bg-amber-800', title: `Espresso: ${espressoVol.toFixed(0)}ml` },
      { percent: pct(iceVol), className: 'bg-sky-300', title: `Ice: ${iceAmount}g` },
      ...(capacityInfo.addonDetails || []).map((a, i) => ({ percent: pct(a.volume), className: `capacity-segment addon-${a.category}`, title: `${a.category}: ${a.volume.toFixed(0)}ml` }))
    ];
    const warning = isExceeded ? (capacityInfo.actualFreeSpace < 0 ? `Exceeds by ${Math.abs(capacityInfo.actualFreeSpace).toFixed(0)}ml` : 'Capacity limit reached') : (!capacityInfo.canAddMore ? 'At max capacity' : null);
    return { segments, freePercent: pct(freeVol), isExceeded, warning };
  };

  const { segments, freePercent, isExceeded, warning } = buildBarSegments();

  const hasMilk = item.selectedMenuItem?.default_ingredients?.some(i => i.category === 'milk');
  const hasIce = item.selectedMenuItem?.default_ingredients?.some(i => i.category === 'ice');
  const hasWater = item.selectedMenuItem?.default_ingredients?.some(i => i.category === 'water');
  const cupType = item.selectedMenuItem?.default_ingredients?.find(i => i.category === 'cups')?.type;
  const isHotMilkDrink = hasMilk && !isIcedCup(cupType);

  /* ── Render ── */
  return (
    <div className="flex flex-col gap-0.5 select-none">

      {/* Capacity bar */}
      {capacityInfo && (
        <CapacityBar segments={segments} freePercent={freePercent} isExceeded={isExceeded} warning={warning} />
      )}

      {/* Temperature */}
      {isHotMilkDrink && (
        <>
          <SectionLabel>Temperature</SectionLabel>
          <PillGroup
            options={TEMPERATURE_OPTIONS}
            selected={selectedTemperature}
            onSelect={handleTemperatureChange}
            getId={o => o.id}
            getLabel={o => o.label}
            getWouldExceed={o => wouldTemperatureExceedCapacity(o.id)}
          />
          <Divider />
        </>
      )}

      {/* Ice */}
      {hasIce && (
        <>
          <AmountStepper
            label="Ice"
            value={iceAmount} unit="g"
            levelLabel={levelLabel(iceAmount, 300)}
            onDecrease={() => handleIceAdjustment(-20)}
            onIncrease={() => handleIceAdjustment(10)}
            decreaseDisabled={iceAmount === 0}
            increaseDisabled={iceAmount >= 300 || capacityInfo?.freeSpaceRemaining <= 0}
            accentClass="bg-sky-500 hover:bg-sky-600 active:bg-sky-700 text-white"
          />
          <Divider />
        </>
      )}

      {/* Milk */}
      {hasMilk && (
        <>
          <AmountStepper
            label="Milk"
            value={milkAmount} unit="g"
            levelLabel={levelLabel(milkAmount, 500)}
            onDecrease={() => handleMilkAdjustment(-10)}
            onIncrease={() => handleMilkAdjustment(10)}
            decreaseDisabled={milkAmount === 0}
            increaseDisabled={milkAmount >= 500 || capacityInfo?.freeSpaceRemaining <= 0}
            accentClass="bg-blue-500 hover:bg-blue-600 active:bg-blue-700 text-white"
          />
          <Divider />
        </>
      )}

      {/* Water */}
      {hasWater && (
        <>
          <AmountStepper
            label="Water"
            value={waterAmount} unit="g"
            levelLabel={levelLabel(waterAmount, 500)}
            onDecrease={() => handleWaterAdjustment(-10)}
            onIncrease={() => handleWaterAdjustment(10)}
            decreaseDisabled={waterAmount === 0}
            increaseDisabled={waterAmount >= 500 || capacityInfo?.freeSpaceRemaining <= 0}
            accentClass="bg-cyan-500 hover:bg-cyan-600 active:bg-cyan-700 text-white"
          />
          <Divider />
        </>
      )}

      {/* Replaceable categories (espresso, milk type) */}
      {item.selectedMenuItem && ingredientsByCategory && typeof ingredientsByCategory === 'object' &&
        Object.entries(ingredientsByCategory).map(([category, ingredients]) => {
          if (hiddenCategories.includes(category.toLowerCase())) return null;
          if (!replaceableCategories.includes(category.toLowerCase())) return null;
          const current = getCurrentSelection(category);
          if (!current) return null;
          const def = item.selectedMenuItem.default_ingredients?.find(d => d.category === category);
          return (
            <div key={category}>
              <SectionLabel>{category.replace('_', ' ')}</SectionLabel>
              <PillGroup
                options={ingredients}
                selected={current.itemId}
                onSelect={(id) => handleReplaceableSelection(category, id)}
                getId={ing => ing.ingredient_id}
                getLabel={ing => ing.name || ing.type}
                getBadge={ing => def && String(current.itemId) === String(ing.ingredient_id) && current.isDefault ? 'default' : null}
              />
              <Divider />
            </div>
          );
        })}

      {/* Additive categories (syrups, sauces, extras) */}
      {item.selectedMenuItem && ingredientsByCategory && typeof ingredientsByCategory === 'object' &&
        Object.entries(ingredientsByCategory).map(([category, ingredients]) => {
          if (hiddenCategories.includes(category.toLowerCase())) return null;
          if (replaceableCategories.includes(category.toLowerCase())) return null;
          if (!ingredients || ingredients.length === 0) return null;
          const addons = getAddonsForCategory(category);
          const defaults = (item.selectedMenuItem?.default_ingredients || []).filter(d => d.category === category);
          if (defaults.length === 0 && addons.length === 0) return null;
          return (
            <div key={category}>
              <div className="flex items-center justify-between mb-1.5">
                <SectionLabel>{category.replace('_', ' ')}</SectionLabel>
                <select
                  className="text-xs border-2 border-gray-200 rounded-xl px-2 py-1.5 bg-white focus:outline-none focus:border-green-500 cursor-pointer disabled:opacity-40 min-h-[36px]"
                  disabled={!canAddMoreAddons()}
                  onChange={(e) => {
                    if (!e.target.value) return;
                    if (!canAddMoreAddons()) { e.target.value = ''; return; }
                    const ex = addons.find(a => String(a.itemId) === String(e.target.value));
                    if (ex) updateAddonQuantity(ex.originalIndex, ex.qty + 1);
                    else addIngredientAddon(itemId, e.target.value, 1);
                    e.target.value = '';
                  }}
                >
                  <option value="">{canAddMoreAddons() ? '+ Add' : 'Full'}</option>
                  {canAddMoreAddons() && ingredients.map(ing => <option key={ing.ingredient_id} value={ing.ingredient_id}>{ing.name}</option>)}
                </select>
              </div>

              {/* Default ingredients (read-only) */}
              {defaults.map((d, i) => (
                <div key={`d-${i}`} className="flex items-center gap-2 px-3 py-2 bg-gray-50 border border-gray-100 rounded-xl mb-1.5">
                  <span className="text-xs text-gray-600 flex-1 truncate">{d.type}</span>
                  <span className="text-[9px] bg-gray-200 text-gray-500 rounded-full px-1.5 py-0.5">default</span>
                </div>
              ))}

              {/* User addons with 32px +/- buttons */}
              {addons.map(addon => {
                const d = getIngredientDetailsById(addon.itemId);
                return (
                  <div key={addon.originalIndex} className="flex items-center justify-between gap-1.5 px-2 py-1.5 bg-green-50 border border-green-100 rounded-xl mb-1.5 overflow-hidden">
                    <span className="text-[7px] text-gray-700 truncate font-bold min-w-0 block pr-1" title={d?.name || getIngredientNameById(addon.itemId)}>
                      {d?.name || getIngredientNameById(addon.itemId)}
                    </span>
                    <div className="flex items-center gap-0.5 flex-shrink-0">
                      <button type="button" onClick={() => updateAddonQuantity(addon.originalIndex, addon.qty - 1)}
                        className="w-7 h-7 rounded-[8px] bg-orange-500 hover:bg-orange-600 active:scale-95 text-white font-bold flex items-center justify-center transition-all leading-none pb-[2px]">−</button>
                      <span className="text-xs font-bold text-gray-700 w-5 text-center">{addon.qty}</span>
                      <button type="button" onClick={() => updateAddonQuantity(addon.originalIndex, addon.qty + 1)}
                        disabled={!canAddMoreAddons()}
                        className="w-7 h-7 rounded-[8px] bg-green-600 hover:bg-green-700 active:scale-95 text-white font-bold flex items-center justify-center transition-all leading-none pb-[1px] disabled:opacity-30">+</button>
                      <button type="button" onClick={() => removeIngredientModification(itemId, addon.originalIndex)}
                        className="w-7 h-7 rounded-[8px] bg-red-400 hover:bg-red-500 active:scale-95 text-white flex items-center justify-center transition-all ml-1">
                        <svg className="w-3.5 h-3.5" fill="none" stroke="currentColor" viewBox="0 0 24 24" strokeWidth="3"><path strokeLinecap="round" strokeLinejoin="round" d="M6 18L18 6M6 6l12 12" /></svg>
                      </button>
                    </div>
                  </div>
                );
              })}
              <Divider />
            </div>
          );
        })}

      {/* Special Instructions — collapsible */}
      <button
        type="button"
        onClick={() => setShowNotes(v => !v)}
        className="flex items-center justify-between w-full min-h-[40px] px-3 rounded-xl hover:bg-green-50 transition-all"
        style={{ border: '2px dashed #9ca3af' }}
      >
        <span className="text-xs font-semibold text-gray-500">
          Special Instructions
          {item.kitchen_notes.filter(n => n.qty > 0).length > 0 && (
            <span className="ml-1.5 text-[10px] bg-green-100 text-green-700 rounded-full px-1.5 py-0.5">
              {item.kitchen_notes.filter(n => n.qty > 0).length}
            </span>
          )}
        </span>
        <svg className={`w-4 h-4 text-gray-400 transition-transform ${showNotes ? 'rotate-180' : ''}`} fill="none" stroke="currentColor" viewBox="0 0 24 24" strokeWidth="2.5">
          <path strokeLinecap="round" strokeLinejoin="round" d="M19 9l-7 7-7-7" />
        </svg>
      </button>

      {showNotes && (
        <div className="mt-1.5 flex flex-col gap-1.5">
          <button
            type="button"
            onClick={() => addKitchenNote(itemId)}
            className="w-full min-h-[40px] text-xs font-bold rounded-xl border-2 border-dashed border-green-400 text-green-600 hover:bg-green-50 active:bg-green-100 transition-colors"
          >
            + Add Note
          </button>
          {item.kitchen_notes.filter(n => n.qty > 0).length === 0 ? (
            <p className="text-[10px] text-gray-400 text-center py-1">No instructions yet</p>
          ) : (
            item.kitchen_notes.filter(n => n.qty > 0).map((note, i) => {
              const realIdx = item.kitchen_notes.findIndex(n => n === note);
              return (
                <div key={i} className="flex gap-1.5 items-center">
                  <input
                    type="text" value={note.type}
                    onChange={e => updateKitchenNote(itemId, realIdx, 'type', e.target.value)}
                    placeholder="e.g. Sugar"
                    className="flex-1 text-xs border-2 border-gray-200 rounded-xl px-2.5 py-2 focus:outline-none focus:border-green-400 min-w-0 min-h-[40px]"
                  />
                  <input
                    type="number" min="1" value={note.qty}
                    onChange={e => updateKitchenNote(itemId, realIdx, 'qty', parseInt(e.target.value) || 1)}
                    className="w-12 text-xs border-2 border-gray-200 rounded-xl px-1.5 py-2 text-center focus:outline-none focus:border-green-400 min-h-[40px]"
                  />
                  <button type="button" onClick={() => removeKitchenNote(itemId, realIdx)}
                    className="w-10 h-10 flex items-center justify-center rounded-xl bg-red-500 hover:bg-red-600 text-white text-base font-bold active:scale-90 transition-all flex-shrink-0">×</button>
                </div>
              );
            })
          )}
        </div>
      )}
    </div>
  );
}
