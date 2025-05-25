// BARNS Design System Theme Configuration
// This file contains all the design tokens, colors, and styling patterns
// for the BARNS Business Automation & Robotics System

export const colors = {
  // Primary Brand Colors
  primary: {
    50: '#f0fdf4',
    100: '#dcfce7',
    200: '#bbf7d0',
    300: '#86efac',
    400: '#4ade80',
    500: '#00754a',  // Main brand color
    600: '#16a34a',
    700: '#15803d',
    800: '#004029',  // Dark brand color
    900: '#14532d',
  },
  
  // Secondary Colors
  secondary: {
    50: '#ecfdf5',
    100: '#d1fae5',
    200: '#a7f3d0',
    300: '#6ee7b7',
    400: '#34d399',
    500: '#22c55e',
    600: '#10b981',
    700: '#059669',
    800: '#065f46',
    900: '#064e3b',
  },
  
  // Status Colors
  success: {
    50: '#f0fdf4',
    100: '#dcfce7',
    500: '#16a34a',
    600: '#15803d',
    700: '#14532d',
  },
  
  warning: {
    50: '#fffbeb',
    100: '#fef3c7',
    500: '#f59e0b',
    600: '#d97706',
    700: '#b45309',
  },
  
  error: {
    50: '#fef2f2',
    100: '#fee2e2',
    500: '#dc2626',
    600: '#b91c1c',
    700: '#991b1b',
  },
  
  // Neutral Colors
  gray: {
    50: '#f9fafb',
    100: '#f3f4f6',
    200: '#e5e7eb',
    300: '#d1d5db',
    400: '#9ca3af',
    500: '#6b7280',
    600: '#4b5563',
    700: '#374151',
    800: '#1f2937',
    900: '#111827',
  },
  
  white: '#ffffff',
  black: '#000000',
};

export const gradients = {
  // Header gradients
  header: 'linear-gradient(135deg, #004029 0%, #00754a 50%, #008552 100%)',
  headerShadow: '0 8px 32px rgba(0, 64, 41, 0.3)',
  
  // Background gradients
  background: 'linear-gradient(135deg, #f0fdf4 0%, #ecfdf5 100%)',
  
  // Button gradients
  primaryButton: 'linear-gradient(135deg, #00754a 0%, #004029 100%)',
  primaryButtonHover: 'linear-gradient(135deg, #004029 0%, #00754a 100%)',
  successButton: 'linear-gradient(135deg, #16a34a 0%, #22c55e 100%)',
  successButtonHover: 'linear-gradient(135deg, #22c55e 0%, #16a34a 100%)',
  
  // Card gradients
  cardSuccess: 'linear-gradient(135deg, #f0fdf4 0%, #dcfce7 100%)',
  cardWarning: 'linear-gradient(135deg, #fffbeb 0%, #fef3c7 100%)',
  cardError: 'linear-gradient(135deg, #fef2f2 0%, #fee2e2 100%)',
};

export const shadows = {
  sm: '0 1px 2px 0 rgba(0, 117, 74, 0.05)',
  base: '0 4px 6px rgba(0, 117, 74, 0.07), 0 1px 3px rgba(0, 117, 74, 0.06)',
  md: '0 8px 15px rgba(0, 117, 74, 0.12), 0 3px 6px rgba(0, 117, 74, 0.08)',
  lg: '0 10px 15px -3px rgba(0, 117, 74, 0.1), 0 4px 6px -2px rgba(0, 117, 74, 0.05)',
  xl: '0 25px 50px -12px rgba(0, 117, 74, 0.25)',
};

export const spacing = {
  xs: '0.5rem',    // 8px
  sm: '0.75rem',   // 12px
  md: '1rem',      // 16px
  lg: '1.5rem',    // 24px
  xl: '2rem',      // 32px
  '2xl': '3rem',   // 48px
  '3xl': '4rem',   // 64px
};

export const borderRadius = {
  sm: '0.375rem',  // 6px
  base: '0.5rem',  // 8px
  md: '0.75rem',   // 12px
  lg: '1rem',      // 16px
  xl: '1.5rem',    // 24px
  full: '9999px',
};

export const typography = {
  fontFamily: "'Inter', 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif",
  fontSizes: {
    xs: '0.75rem',
    sm: '0.875rem',
    base: '1rem',
    lg: '1.125rem',
    xl: '1.25rem',
    '2xl': '1.5rem',
    '3xl': '1.875rem',
    '4xl': '2.25rem',
  },
  fontWeights: {
    normal: '400',
    medium: '500',
    semibold: '600',
    bold: '700',
  },
};

// Component Style Presets
export const components = {
  button: {
    base: `
      border-radius: ${borderRadius.base};
      border: 1px solid transparent;
      padding: 0.6em 1.2em;
      font-size: 1em;
      font-weight: 500;
      font-family: inherit;
      cursor: pointer;
      transition: all 0.3s ease;
      box-shadow: ${shadows.sm};
    `,
    primary: `
      background: ${gradients.primaryButton};
      color: ${colors.white};
      border: none;
    `,
    secondary: `
      background: ${colors.gray[100]};
      color: ${colors.gray[800]};
      border: 1px solid ${colors.gray[300]};
    `,
    success: `
      background: ${gradients.successButton};
      color: ${colors.white};
      border: none;
    `,
  },
  
  card: {
    base: `
      background: ${colors.white};
      border: 1px solid ${colors.gray[200]};
      border-radius: ${borderRadius.md};
      box-shadow: ${shadows.base};
      transition: all 0.3s ease;
    `,
    elevated: `
      box-shadow: ${shadows.md};
    `,
  },
  
  badge: {
    base: `
      display: inline-flex;
      align-items: center;
      padding: 0.25rem 0.75rem;
      border-radius: ${borderRadius.full};
      font-size: ${typography.fontSizes.xs};
      font-weight: ${typography.fontWeights.medium};
      line-height: 1;
    `,
    success: `
      background-color: rgba(22, 163, 74, 0.1);
      color: ${colors.success[600]};
    `,
    warning: `
      background-color: rgba(245, 158, 11, 0.1);
      color: ${colors.warning[600]};
    `,
    error: `
      background-color: rgba(220, 38, 38, 0.1);
      color: ${colors.error[600]};
    `,
    primary: `
      background-color: rgba(0, 117, 74, 0.1);
      color: ${colors.primary[500]};
    `,
  },
};

// Animation presets
export const animations = {
  fadeIn: 'fadeIn 0.3s ease-in-out',
  slideIn: 'slideIn 0.3s ease-out',
  pulse: 'barns-pulse 2s ease-in-out infinite',
  glow: 'barns-glow 3s ease-in-out infinite',
};

// Breakpoints for responsive design
export const breakpoints = {
  sm: '640px',
  md: '768px',
  lg: '1024px',
  xl: '1280px',
  '2xl': '1536px',
};

// Z-index scale
export const zIndex = {
  dropdown: 1000,
  sticky: 1020,
  fixed: 1030,
  modalBackdrop: 1040,
  modal: 1050,
  popover: 1060,
  tooltip: 1070,
};

export default {
  colors,
  gradients,
  shadows,
  spacing,
  borderRadius,
  typography,
  components,
  animations,
  breakpoints,
  zIndex,
}; 