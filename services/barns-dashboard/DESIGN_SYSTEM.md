# BARNS Design System

## Overview

The BARNS Design System is a comprehensive design language for the Business Automation & Robotics System dashboard. It provides a cohesive visual identity centered around the BARNS brand green color (`#00754a`) and modern, professional aesthetics.

## Brand Identity

- **Primary Color**: `#00754a` (BARNS Green)
- **Logo**: BARNS-Logo.png (integrated in header)
- **Theme**: Modern, tech-focused, professional
- **Typography**: Inter font family for clean, readable interface

## Design Tokens

All design tokens are centralized in `src/theme.js` for consistency and maintainability.

### Colors

#### Primary Brand Colors
```javascript
primary: {
  500: '#00754a',  // Main brand color
  800: '#004029',  // Dark brand color
  // ... full spectrum available
}
```

#### Status Colors
- **Success**: Green spectrum for positive states
- **Warning**: Amber for caution states  
- **Error**: Red for error states
- **Neutral**: Gray spectrum for text and borders

### CSS Custom Properties

The design system uses CSS custom properties defined in `src/index.css`:

```css
:root {
  --barns-primary: #00754a;
  --barns-primary-dark: #004029;
  --barns-success: #16a34a;
  --barns-warning: #f59e0b;
  --barns-error: #dc2626;
  /* ... full set of variables */
}
```

## Component Library

### Buttons

#### CSS Classes
- `.btn-primary` - Primary green gradient button
- `.btn-secondary` - Light gray secondary button  
- `.btn-success` - Success green gradient button

#### Usage Example
```jsx
<button className="btn-primary">
  Primary Action
</button>
```

### Cards

#### CSS Classes
- `.card` - Base card styling with white background and subtle shadow
- `.card-elevated` - Enhanced shadow for important cards

#### Usage Example
```jsx
<div className="card card-elevated">
  <div className="p-6">
    Card content
  </div>
</div>
```

### Badges

#### CSS Classes
- `.badge` - Base badge styling
- `.badge-success` - Green success badge
- `.badge-warning` - Amber warning badge
- `.badge-error` - Red error badge
- `.badge-primary` - Primary brand color badge

#### Usage Example
```jsx
<span className="badge badge-success">
  Online
</span>
```

### Status Indicators

#### CSS Classes
- `.status-success` - Green text and background for positive states
- `.status-warning` - Amber text and background for warnings
- `.status-error` - Red text and background for errors
- `.status-pending` - Gray text and background for neutral states

## Layout System

### Spacing Scale
- `xs`: 8px
- `sm`: 12px  
- `md`: 16px
- `lg`: 24px
- `xl`: 32px
- `2xl`: 48px
- `3xl`: 64px

### Responsive Breakpoints
- `sm`: 640px
- `md`: 768px
- `lg`: 1024px
- `xl`: 1280px
- `2xl`: 1536px

## Typography

### Font Stack
```css
font-family: 'Inter', 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
```

### Font Weights
- Normal: 400
- Medium: 500
- Semibold: 600
- Bold: 700

### Usage Guidelines
- Use semibold (600) for headings
- Use medium (500) for body text
- Use bold (700) for important emphasis

## Shadows and Depth

The design system uses a consistent shadow scale with the brand green tint:

- `sm`: Subtle shadow for buttons
- `base`: Default card shadow
- `md`: Elevated card shadow
- `lg`: Modal and dropdown shadow
- `xl`: Large modal shadow

## Animation and Interactions

### Keyframe Animations
- `barns-pulse`: Gentle pulsing for loading states
- `barns-glow`: Subtle glow effect for active elements

### Transitions
- Standard duration: 300ms
- Easing: `ease` or `ease-in-out`

## Header Design

The header features:
- BARNS logo in a frosted glass container
- Green gradient background with the brand colors
- White active tab indicators with green text
- Enhanced shadow for depth

## Best Practices

### Color Usage
1. **Primary green** for main actions and brand elements
2. **Secondary green** for supporting actions
3. **Success green** for positive feedback
4. **Neutral grays** for text and borders
5. **Semantic colors** (warning amber, error red) for system states

### Component Composition
1. Use the `card` class for content containers
2. Apply `card-elevated` for important sections
3. Use badge components for status indicators
4. Follow the button hierarchy (primary > secondary > success)

### Accessibility
- Maintain WCAG 2.1 AA contrast standards
- Use semantic HTML elements
- Provide clear focus indicators
- Ensure keyboard navigation works properly

## File Structure

```
src/
├── theme.js              # Design system configuration
├── index.css             # Global styles and CSS custom properties
├── assets/
│   └── BARNS-Logo.png    # Brand logo
└── components/           # Component implementations
```

## Implementation Examples

### Import and Use Theme
```javascript
import theme from '../theme';

// Use in styled components or inline styles
const cardStyle = {
  background: theme.colors.white,
  boxShadow: theme.shadows.base,
  borderRadius: theme.borderRadius.md,
};
```

### System Status Component
```jsx
<div className="flex items-center justify-between p-3 bg-white bg-opacity-70 rounded-lg">
  <div className="flex items-center space-x-3">
    <div className="w-3 h-3 rounded-full bg-green-500 shadow-sm"></div>
    <span className="font-medium text-gray-700">Service Name</span>
  </div>
  <span className="badge badge-success">Online</span>
</div>
```

## Future Considerations

- Dark mode support (additional color tokens)
- Component library extraction for reuse
- Figma design system integration
- Advanced animation library integration

---

This design system ensures consistency, maintainability, and a professional appearance across the BARNS dashboard while staying true to the brand identity. 