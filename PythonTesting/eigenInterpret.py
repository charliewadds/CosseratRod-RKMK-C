import numpy as np
import pandas as pd

def interpret_eigenvalues(eigenvalues):
    real_parts = eigenvalues.real
    imag_parts = eigenvalues.imag

    if np.all(real_parts < 0):
        if np.any(imag_parts != 0):
            return "Stable Focus"
        else:
            return "Stable Node"
    elif np.all(real_parts > 0):
        if np.any(imag_parts != 0):
            return "Unstable Focus"
        else:
            return "Unstable Node"
    elif np.any(real_parts > 0) and np.any(real_parts < 0):
        return "Saddle Point"
    elif np.all(real_parts == 0) and np.any(imag_parts != 0):
        return "Center"
    else:
        return "Unknown"

def process_jacobians_from_csv(csv_file):
    # Read the CSV file
    df = pd.read_csv(csv_file, header=None)

    # Assuming each 6x6 Jacobian matrix is stacked vertically in the CSV
    num_matrices = len(df) // 6
    results = []

    for i in range(num_matrices):
        start_row = i * 6
        end_row = start_row + 6
        jacobian_matrix = df.iloc[start_row:end_row].values

        # Compute eigenvalues and eigenvectors
        eigenvalues, eigenvectors = np.linalg.eig(jacobian_matrix)

        # Interpret the eigenvalues
        interpretation = interpret_eigenvalues(eigenvalues)

        # Append the results
        results.append({
            "Matrix Index": i,
            "Eigenvalues": eigenvalues,
            "Eigenvectors": eigenvectors,
            "Interpretation": interpretation
        })

    return results

def main():
    csv_file = "../cmake-build-debug/jacobian6.csv"  # Replace with your actual CSV file path
    results = process_jacobians_from_csv(csv_file)

    # Print the results
    for result in results:
        print(f"Matrix Index: {result['Matrix Index']}")
        print(f"Eigenvalues: {result['Eigenvalues']}")
        print(f"Eigenvectors: \n{result['Eigenvectors']}")
        print(f"Interpretation: {result['Interpretation']}\n")

if __name__ == "__main__":
    main()
